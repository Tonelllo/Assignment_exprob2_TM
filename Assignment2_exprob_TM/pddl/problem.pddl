(define (problem pippo) (:domain robot_navigation)
(:objects 
    robot - robot
    waypoint - waypoint
)

(:init
    (at-robby robot base)
    ;todo: put the initial state's facts and numeric values here
)

(:goal (and (explored waypoint)
    ;todo: put the goal condition here
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)

