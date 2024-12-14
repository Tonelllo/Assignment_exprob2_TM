(define (domain robot_navigation)

(:requirements :strips :fluents :adl :durative-actions :typing)

(:types 
    robot
    location 
    waypoint - location
)


(:predicates 
    (at-robby ?r - robot ?l - location)
    (explored ?w - waypoint) 
)

(:durative-action move
    :parameters (?r - robot ?l1 ?l2 - location)
    :duration (= ?duration 5)
    :condition (and 
        (at start ((at-robby ?r ?l1)))
        ;(at start (not (explored ?l2)))
    )
    :effect (and 
        (at start ((not (at-robby ?r ?l1))))
        (at end ((at-robby ?r ?l2)))
    )
)

(:durative-action explore_waypoint
    :parameters (?r - robot ?w - waypoint)
    :duration (= ?duration 5)
    :condition (and 
        (at start ((at-robby ?r ?w)))
        (at start (not (explored ?w)))            
    )
    :effect (and 
        (at end (explored ?w))
    )
)
)
