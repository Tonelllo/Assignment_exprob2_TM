(define (domain robot_navigation)
(:requirements :strips :typing :durative-actions)
(:types 
    robot
    waypoint 
)

(:predicates 
    (at-robby ?r - robot ?w - waypoint)
    (explored ?w - waypoint) 
    (to_go ?w - waypoint)
    (min ?w - waypoint)
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

(:durative-action move_to_min
  :parameters (?r - robot ?w1 - waypoint ?w2 - waypoint)
  :duration (= ?duration 5)
  :condition (and
      (at start (at-robby ?r ?w1))
      (at start (explored ?w1))
      (at start (explored ?w2))
      )
  :effect (and
      (at start (not (at-robby ?r ?w1)))
      (at end (at-robby ?r ?w2))
      (at end (min ?w2))
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
)