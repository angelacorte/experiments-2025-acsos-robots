package it.unibo.collektive.program

import it.unibo.alchemist.model.sensors.DepotsSensor
import it.unibo.alchemist.model.sensors.LocationSensor
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.formalization.Bid
import it.unibo.formalization.RoutingHeuristics
import it.unibo.formalization.Node as NodeFormalization

data class ReplanningState(
    val dones: Map<NodeFormalization, Boolean>,
    val assigned: Set<NodeFormalization>,
    val path: List<NodeFormalization>
) {
    companion object {
        fun createFrom(tasks: Set<NodeFormalization>, depotsSensor: DepotsSensor): ReplanningState {
            val assigned = setOf<NodeFormalization>()
            val path = listOf(depotsSensor.sourceDepot, depotsSensor.destinationDepot)
            return ReplanningState(
                dones = tasks.associate { it to false }.toMap(),
                assigned = assigned,
                path = path
            )
        }
    }
}

fun Aggregate<Int>.everDone(dones: Map<NodeFormalization, Boolean>): Map<NodeFormalization, Boolean> {
    val others = neighboring(dones)
    val neighbourhood = others.toMap()
    val updated = neighbourhood.values.reduce {
            acc, v -> acc.map { (key, value) -> key to (value || v[key] ?: false) }.toMap()
    }
    return updated
}

val balanceFactor = 100
fun Aggregate<Int>.fullRuntime(
    env: EnvironmentVariables,
    locationSensor: LocationSensor,
    depotsSensor: DepotsSensor
) {
    env["hue"] = localId // for debugging
    if(!depotsSensor.alive()) {
        env["target"] = locationSensor.coordinates()
    } else {
        val allTasks = depotsSensor.tasks.toSet()
        val myPosition = locationSensor.coordinates()
        val initialPosition = evolve(myPosition) { it}
        evolve(ReplanningState.createFrom(allTasks, depotsSensor)) { state ->
            /** Ever done -- check if the task is completed by one of the robot */
            val allTaskDone = everDone(state.dones)
            /** BIDDING PROCESS -- compute the bids */
            val adaptMarginalCostByAssignedFactor = 1 + state.assigned.size * balanceFactor
            /** - remove the same tasks in the field in order to rebid */
            val sameTasksInField = neighboring(state.assigned).toMap().map {
                (key, tasks) -> tasks.map { key to it }}.flatten()
            val sameTasksGroupById = sameTasksInField.groupBy { it.second.id }
            // for each task, take the one with the lowest node id
            val takeWinner = sameTasksGroupById.map { (_, node) -> node.minBy { it.first } }
            val takeTaskToRemove = takeWinner.filter { it.first != localId }
            // remove the tasks from the path
            val marginal = (allTasks - state.dones.filter { it.value }.keys).map { task ->
                // filter the path if it contains the task
                val pathWithoutTask = state.path.filterNot { it.id == task.id }
                task to RoutingHeuristics.computeMarginalCost(pathWithoutTask, task) * adaptMarginalCostByAssignedFactor
            }.map { (task, cost) -> Bid(task, localId, cost) }
            val taskBids = neighboring(marginal).toMap().values
                .flatten()
                .groupBy { it.task }
                .mapValues { (_, bids) -> bids.toMutableList() }
            env["bids"] = taskBids.map { (task, bids) ->
                task.id to bids.map { it.robot }
            }.toMap()
            /** TASK ALLOCATION -- take the best bid */
            val assignedLocal = taskBids
                .filter { (_, bids) -> bids.isNotEmpty() }
                .firstNotNullOfOrNull { (task, bids) ->
                    val bestBid = bids.minWithOrNull(compareBy<Bid> { it.cost }.thenBy { it.robot })
                    if (bestBid?.robot == localId) task else null
                }
                ?.run {
                    (state.assigned - takeTaskToRemove.map { it.second }.toSet()) + this
                } ?: state.assigned
            val filterDones = assignedLocal - state.dones.filter { it.value }.keys
            // Store the updated path in environment variables
            val updatedPath = RoutingHeuristics.solveLocalRouting(filterDones.toSet(), NodeFormalization(myPosition, localId), depotsSensor.destinationDepot)
            env["raw"] = updatedPath.map { it.id }

            val updated = state.copy(dones = allTaskDone, assigned = assignedLocal, path = updatedPath)
            followPlan(
                env,
                depotsSensor,
                locationSensor,
                updated
            )
        }
    }
}

fun Aggregate<Int>.followPlan(
    env: EnvironmentVariables,
    depotsSensor: DepotsSensor,
    locationSensor: LocationSensor,
    state: ReplanningState
): ReplanningState {
    val path = state.path.drop(1) // source depot is not a task
    // find first available
    val firstAvailable = path.firstOrNull { !(state.dones[it] ?: false) }
    val selected = firstAvailable ?: state.path.last()

    env["target"] = locationSensor.estimateCoordinates(selected)
    env["selected"] = selected.id
    // check if the task is done
    val isDone = depotsSensor.isTaskOver(selected)
    val updated = state.dones.toMutableMap()
    updated[selected] = isDone
    // update the state
    return state.copy(dones = updated)
}
