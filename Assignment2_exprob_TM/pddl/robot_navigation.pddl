;Header and description

(define (domain robot_navigation)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot
    location 
    waypoint - location
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (at-robby ?l - location)
    (explored ?w - waypoint) 
)


(:functions ;todo: define numeric functions here
)

(:durative-action move
    :parameters (?r - robot ?l1 ?l2 - location)
    :duration (= ?duration 1)
    :condition (and 
        (at start ((at-robby ?l1)))
        (at start (not (explored ?l2)))
    )
    :effect (and 
        (at start ((not (at-robby ?l1))))
        (at end ((at-robby ?l2)))
    )
)

(:durative-action explore_waypoint
    :parameters (?r - robot ?w - waypoint)
    :duration (= ?duration 1)
    :condition (and 
        (at start ((at-robby ?w)))
        (at start (not (explored ?w)))            
    )
        ; TROVARE MODO PER DIRGLI DI CERCARE MARKER

    :effect (and 
        (at end (explored ?w))
    )
)
)
