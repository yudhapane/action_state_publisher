(define (problem prob0)

  (:domain packaging)

  (:objects 
            left-arm right-arm - arm
            surface-left surface-right ; - surface
            handover-left handover-right ; - space
            bin ; - container
            package ; - package
            camera ; - camera
            obj1 obj2 obj3 obj4 obj5 obj6 - location ; - graspable
            )

  (:init 
            ;; location
            (arm left-arm)
            (arm right-arm)
            (surface surface-left)
            (surface surface-right)
            (space handover-left)
            (space handover-right)
            (container bin)
            (package package)
            (camera camera)

            ;; graspable
            (graspable obj1)
            (graspable obj2)
            (graspable obj3)
            (graspable obj4)
            (graspable obj5)
            (graspable obj6)

            (unknown_orientation obj1)
            (unknown_orientation obj2)
            (unknown_orientation obj3)
            (unknown_orientation obj4)
            (unknown_orientation obj5)
            (unknown_orientation obj6)

            ;; arm_canreach
            (arm_canreach left-arm handover-left)
            (arm_canreach left-arm surface-left)
            (arm_canreach left-arm bin)
            (arm_canreach left-arm camera)
            (arm_canreach left-arm package)
            (arm_canreach left-arm obj1)
            (arm_canreach left-arm obj2)
            (arm_canreach left-arm obj3)
            (arm_canreach left-arm obj4)
            (arm_canreach left-arm obj5)
            (arm_canreach left-arm obj6)

            (arm_canreach right-arm handover-right)
            (arm_canreach right-arm surface-right)
            (arm_canreach right-arm bin)
            (arm_canreach right-arm camera)
            (arm_canreach right-arm package)
            (arm_canreach right-arm obj1)
            (arm_canreach right-arm obj2)
            (arm_canreach right-arm obj3)
            (arm_canreach right-arm obj4)
            (arm_canreach right-arm obj5)
            (arm_canreach right-arm obj6)

            ;; arm_free
            (arm_free left-arm)
            (arm_free right-arm)

            ;; arm_at
            (arm_at left-arm handover-left)
            (arm_at right-arm handover-right)

            ;; object_in
            (object_in obj1 bin)
            (object_in obj2 bin)
            (object_in obj3 bin)
            (object_in obj4 bin)
            (object_in obj5 bin)
            (object_in obj6 bin)
 
            ;; location free state
            (location_free surface-left)
            (location_free surface-right)
            (location_free bin)
            (location_free package)
            (location_free camera)
            (location_free obj1)
            (location_free obj2)
            (location_free obj3)
            (location_free obj4)
            (location_free obj5)
            (location_free obj6)
 
            ;; obstruction state
            ; (blocked obj1 obj2)
            (unblocked obj1)
            (unblocked obj2)
            (unblocked obj3)
            (unblocked obj4)
            (unblocked obj5)
            (unblocked obj6)
 
            ;; improper grasp pose
            ; (improper_grasp right-arm obj1)
            )

  (:goal (and
            (packed obj1 package)
            (packed obj2 package)
            (packed obj3 package)
            (packed obj4 package)
            (packed obj5 package)
            (packed obj6 package)
          ))

)