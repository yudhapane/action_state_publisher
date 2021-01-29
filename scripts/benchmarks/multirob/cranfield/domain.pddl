(define (domain cranfield)

  (:requirements :strips :typing :conditional-effects :probabilistic-effects)

  (:types container assemble camera - location 
          stand - container 
          graspable arm)

  (:predicates  (free ?o - location)

                (arm_canreach ?a - arm ?l - location)
                (arm_at ?a - arm ?l - location)
                (arm_vacuumed ?a - arm ?o - graspable)
                (arm_gripped ?a - arm ?o - graspable)
                (arm_free ?a - arm)

                (object_in ?o - graspable ?c - location)
                (object_at ?o - graspable ?c - location)
                (unknown_orientation ?o - graspable)
                (downward ?o - graspable)
                (upward ?o - graspable)
                (ungripped ?o - graspable)

                (assembled ?o - graspable ?c - graspable)
                (packed ?o1 - graspable ?o2 - graspable ?c - location))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; robot actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; move/carry actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:action move_to_grasp
 :parameters   (?a - arm ?s ?d - location ?o - graspable)
 :precondition (and (arm_free ?a) (arm_at ?a ?s) (arm_canreach ?a ?d) (free ?d) (object_in ?o ?d))
 :effect       (and (arm_at ?a ?d) (not (arm_at ?a ?s)) (free ?s) (not (free ?d))))

(:action carry_to_assemble
 :parameters   (?a - arm ?s ?d - location ?o - graspable)
 :precondition (and (arm_gripped ?a ?o) (arm_at ?a ?s) (arm_canreach ?a ?d) (free ?d))
 :effect       (and (arm_at ?a ?d) (not (arm_at ?a ?s)) (free ?s) (not (free ?d))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; vacuum/grip actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:action grasp_object
 :parameters   (?a - arm ?o - graspable ?l - location)
 :precondition (and (arm_free ?a) (arm_at ?a ?l) (object_at ?o ?l))
 :effect       (and (arm_gripped ?a ?o) (not (arm_free ?a)) (not (object_at ?o ?l))))

(:action release_object
 :parameters   (?a - arm ?o - graspable)
 :precondition (and (arm_gripped ?a ?o))
 :effect       (and (arm_free ?a) (ungripped ?o) (not (arm_gripped ?a ?o))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; put/place actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:action insert_object
 :parameters   (?a - arm ?o - graspable ?l - container)
 :precondition (and (arm_gripped ?a ?o) (arm_at ?a ?l) (downward ?o))
 :effect       (and (arm_free ?a) (object_at ?o ?l) (not (arm_gripped ?a ?o)) (not (downward ?o))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; camera actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:action check_orientation
 :parameters   (?a - arm ?o - graspable ?c - camera)
 :precondition (and (arm_vacuumed ?a ?o) (arm_at ?a ?c) (unknown_orientation ?o))
 :effect (oneof (and (downward ?o) (not (unknown_orientation ?o)))
                (and (upward ?o) (not (unknown_orientation ?o)))))

(:action rotate
 :parameters   (?a - arm ?o - graspable)
 :precondition (and (arm_vacuumed ?a ?o) (upward ?o))
 :effect (and (not (upward ?o)) (downward ?o)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; assemble actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:action assemble
 :parameters   (?a1 ?a2 - arm ?o1 ?o2 - graspable ?l1 ?l2 - assemble)
 :precondition (and (arm_gripped ?a1 ?o1) (arm_at ?a1 ?l1) 
                    (arm_gripped ?a2 ?o2) (arm_at ?a2 ?l2))
 :effect       (and (assembled ?o1 ?o2)))

)