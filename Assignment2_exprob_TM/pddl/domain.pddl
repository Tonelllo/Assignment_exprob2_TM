(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?ro - waypoint)
(visited ?r - robot ?ro - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1))
        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)


(:durative-action visit
    :parameters (?r - robot ?ro - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?ro))
    )
    :effect (and
         (at end(robot_at ?r ?ro))
         (at end(visited ?r ?ro))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
