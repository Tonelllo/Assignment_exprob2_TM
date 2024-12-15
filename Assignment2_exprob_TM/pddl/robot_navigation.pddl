(define (domain robot_navigation)
(:requirements :strips :typing :durative-actions)
(:types 
    robot
    location 
    waypoint - location
)

(:predicates 
    (at-robby ?r - robot ?l - location)
    (explored ?l - location) 
)

(:durative-action move
    :parameters (?r - robot ?l1 ?l2 - location)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at-robby ?r ?l1))
        ;(at start (not (explored ?l2)))
    )
    :effect (and 
        (at start (not (at-robby ?r ?l1)))
        (at end (at-robby ?r ?l2))
    )
)

(:durative-action explore_location
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 5)
    :condition (and 
        (at start (at-robby ?r ?l))
        ;(at start (not (explored ?l)))            
    )
    :effect (and 
        (at end (explored ?l))
    )
)
)