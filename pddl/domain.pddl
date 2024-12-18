(define (domain robot_navigation)
(:requirements :strips :typing :durative-actions)
(:types 
    robot
    waypoint 
)

(:predicates 
    (at-robby ?r - robot ?w - waypoint)
    (explored ?w - waypoint) 
)

(:durative-action move
    :parameters (?r - robot ?w1 ?w2 - waypoint)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at-robby ?r ?w1))
        ;(at start (not (explored ?w2)))
    )
    :effect (and 
        (at start (not (at-robby ?r ?w1)))
        (at end (at-robby ?r ?w2))
    )
)

(:durative-action explore_waypoint
    :parameters (?r - robot ?w - waypoint)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at-robby ?r ?w))
        ;(at start (not (explored ?w)))            
    )
    :effect (and 
        (at end (explored ?w))
    )
)
)