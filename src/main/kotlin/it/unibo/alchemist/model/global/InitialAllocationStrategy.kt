package it.unibo.alchemist.model.global

import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.TimeDistribution
import it.unibo.alchemist.model.implementations.reactions.AbstractGlobalReaction
import it.unibo.alchemist.model.molecules.SimpleMolecule

private const val TASKS_MOLECULE = "tasks"
private const val TASK_MOLECULE = "task"
private const val DEPOT_MOLECULE = "depot"
private const val FINISH = "finish"
data class Allocation<T>(val robot: Node<T>, val tasks: List<Node<T>>)

abstract class InitialAllocationStrategy<T, P: Position<P>>(
    environment: Environment<T, P>,
    timeDistribution: TimeDistribution<T>
): AbstractGlobalReaction<T, P>(environment, timeDistribution) {
    override fun executeBeforeUpdateDistribution() {
        val robots = nodes.filter { it.contents[SimpleMolecule(TASK_MOLECULE)] == null }
        val tasks: List<Node<T>> = nodes.filter { it.contents[SimpleMolecule(TASK_MOLECULE)] == true }
        val depot: Node<T> = nodes.first { it.contents[SimpleMolecule(DEPOT_MOLECULE)] == true }
        val allocations = allocate(robots, tasks)
        allocations.forEach {
            it.robot.setConcentration(
                SimpleMolecule(TASKS_MOLECULE),
                (it.tasks + listOf(depot)) as T
            )
        }
    }

    protected abstract fun allocate(robots: List<Node<T>>, tasks: List<Node<T>>): List<Allocation<T>>
}