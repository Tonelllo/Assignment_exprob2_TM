(define (domain robot_navigation)
(:requirements :strips :typing :durative-actions)
(:types 
    robot
    waypoint 
)

(:predicates 
    (at-robby ?r - robot ?w - waypoint)
    (explored ?w - waypoint) 
    (comes_after ?w1 - waypoint ?w2 - waypoint)
    (patrolled ?w - waypoint)
    (to_go ?w - waypoint)
)

(:durative-action move
    :parameters (?r - robot ?w1 - waypoint ?w2 - waypoint)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at-robby ?r ?w1))
        (at start (to_go ?w2))
        )
    :effect (and 
        (at start (not (at-robby ?r ?w1)))
        (at end (at-robby ?r ?w2))
        (at end (not (to_go ?w2)))
        )
)

(:durative-action explore_waypoint
    :parameters (?r - robot ?w - waypoint)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at-robby ?r ?w)))
    :effect (and 
        (at end (explored ?w)))
)

(:durative-action move_with_order
    :parameters (?r - robot ?w1 - waypoint ?w2 - waypoint)
    :duration (= ?duration 5)
    :condition (and
        ;; [plansys2_node-11] terminate called after throwing an instance of 'parser::pddl::UnsupportedConstruct'
        ;; [plansys2_node-11]   what():  Forall is not currently supported by plansys2
        ;;(at start (forall (?w - waypoint) (explored ?w)))
        (at start (explored ?w1))
        (at start (explored ?w2))
        (at start (comes_after ?w1 ?w2))
        (at start (at-robby ?r ?w1))
        )
    :effect (and
        (at start (not (at-robby ?r ?w1)))
        (at end (at-robby ?r ?w2))
        (at end (patrolled ?w2))
        )
)

)