import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch  # noqa: E402
import launch.actions  # noqa: E402
import launch.events  # noqa: E402

import launch_ros.actions  # noqa: E402
import launch_ros.events  # noqa: E402
import launch_ros.events.lifecycle  # noqa: E402

import lifecycle_msgs.msg  # noqa: E402


def generate_launch_description():
    """Run lifecycle nodes via launch."""
    ld = launch.LaunchDescription()

    # Prepare the sim_robots node.
    sim_robots_node = launch_ros.actions.LifecycleNode(
        package='patrol_sim_robots', executable='sim_robots_node', output='screen',
        name='sim_robots_node',
        parameters=[
            {'test': 'hello'}
            # config
        ],),

    # Prepare the robot_1 node.
    robot_node = launch_ros.actions.LifecycleNode(
        name='robot_node', namespace='robot',
        # package='patrol_robot', executable='robot_node', output='screen')
        package='patrol_robot', executable='robot_node')

    # When the sim_robots reaches the 'inactive' state, make it take the 'activate' transition and start the robot node.
    register_event_handler_for_sim_robots_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=sim_robots_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'sim_robots' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(sim_robots_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),

            ],
        )
    )

    # When the sim_robots node reaches the 'active' state, log a message
    register_event_handler_for_sim_robots_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=sim_robots_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'sim_robots' reached the 'active' state, launching a robot."),
                # robot_node,
            ],
        )
    )

    # Make the sim_robots node take the 'configure' transition.
    emit_event_to_request_that_sim_robots_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(sim_robots_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_sim_robots_reaches_inactive_state)
    ld.add_action(register_event_handler_for_sim_robots_reaches_active_state)
    ld.add_action(sim_robots_node)
    ld.add_action(emit_event_to_request_that_sim_robots_does_configure_transition)

    print('Starting introspection of launch description...')
    print('')

    print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    return ld
