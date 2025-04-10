package it.unibo.collektive.program

import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.sensors.DepotsSensor
import it.unibo.alchemist.model.sensors.LocationSensor
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import it.unibo.formalization.GreedyAllocationStrategy
import it.unibo.formalization.Node as NodeFormalization

fun Aggregate<Int>.followTasks(env: EnvironmentVariables, locationSensor: LocationSensor, depotsSensor: DepotsSensor) {
    val tasks = env.get<List<NodeFormalization>>("tasks")
    evolve(tasks.map { false }.toMutableList()) { dones ->
        val firstIndexFalse = dones.indexOfFirst { !it }
        val task = tasks[firstIndexFalse]
        env["target"] = locationSensor.estimateCoordinates(task)
        env["selected"] = task.id
        val isDone = depotsSensor.isTaskOver(task)
        // update the task done
        dones[firstIndexFalse] = isDone
        dones
    }

}

fun Aggregate<Int>.runtimeReconfiguration(
    env: EnvironmentVariables,
    locationSensor: LocationSensor,
    depotsSensor: DepotsSensor
) {
    if(depotsSensor.isAgent()) {
        val tasks = env.getOrNull<List<Node<Any>>>("tasks") ?: emptyList()
        val allNeighborhood = neighboring(tasks).toMap()
        val mockNeighboring = neighboring(listOf<Node<Any>>())
        val remember = evolve(mockNeighboring.toMap()) { allNeighborhood.toMap() }
        evolve(0) {
            neighboring(it).localValue
        }
        val myPosition = locationSensor.coordinates().let { Pair(it.first, it.second) }
        val robots = neighboring(localId to myPosition).toMap().toSortedMap().values.toList().map {
            NodeFormalization(it.second, it.first)
        }
        // flatten on all the tasks
        val start = depotsSensor.sourceDepot
        val destination = depotsSensor.destinationDepot
        val allTasks = depotsSensor.tasks
        val allocator = GreedyAllocationStrategy(
            robots,
            allTasks,
            start,
            destination,
        )
        val result = allocator.execute()
        result.find { it.robot.id == localId }?.let { myTasks ->
            // remove source
            val path = myTasks.route.filterNot { it == start }
            env["raw"] = path
            env["tasks"] = path
        }

        followTasks(env, locationSensor, depotsSensor)
    }
}
