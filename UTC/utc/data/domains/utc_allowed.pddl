(define (domain utc_allowed)
    (:requirements :typing)

    (:types road junction car use)

    (:predicates
        (togo ?c - car ?dest - junction) ; destination position of vehicle
        (connected ?junction1 - junction ?road - road ?junction2 - junction) ; connection between junctions (by roads)
        (allowed ?c - car ?r - road) ; if vehicle is allowed to use road
        (at ?car - car ?junction - junction) ; current car location on junction
        (next ?x ?y - use) ; increasing the current usage
        (light ?r - road ?u - use) ; light capacity of road based on current usage
        (medium ?r - road ?u - use) ; medium capacity of road based on current usage
        (heavy ?r - road ?u - use) ; heavy capacity of road based on current usage
        (cap ?r - road ?u - use) ; maximal capacity of road
        (using ?r - road ?u - use) ; current capacity of road
    )

    (:functions
        (length-light ?road - road) ; time needed to drive pass the road in light traffic
        (length-medium ?road - road) ; time needed to drive pass the road in medium traffic
        (length-heavy ?road - road) ; time needed to drive pass the road in heavy traffic
        (total-cost) ; sum of all actions cost
    )

    ; Actions for vehicle movement between junctions, depending on current capacity

    (:action DRIVE-TO-light
        :parameters (?c - car ?j1 - junction ?r - road ?j2 - junction ?d - junction ?u1 ?u2 - use)
            :precondition (and (togo ?c ?d)
                (allowed ?c ?r)
                (using ?r ?u1)
                (next ?u1 ?u2)
                (light ?r ?u2)
                (at ?c ?j1)
                (connected ?j1 ?r ?j2)
            )

    :effect (and (not (using ?r ?u1))
            (using ?r ?u2)
            (not (at ?c ?j1))
            (at ?c ?j2)
            (increase (total-cost) (length-light ?r))
        )
    )


    (:action DRIVE-TO-medium
        :parameters (?c - car ?j1 - junction ?r - road ?j2 - junction ?d - junction ?u1 ?u2 - use)
            :precondition (and (togo ?c ?d)
                (allowed ?c ?r)
                (using ?r ?u1)
                (next ?u1 ?u2)
                (medium ?r ?u2)
                (at ?c ?j1)
                (connected ?j1 ?r ?j2)
            )

    :effect (and (not (using ?r ?u1))
            (using ?r ?u2)
            (not (at ?c ?j1))
            (at ?c ?j2)
            (increase (total-cost) (length-medium ?r))
        )
    )


    (:action DRIVE-TO-heavy
        :parameters (?c - car ?j1 - junction ?r - road ?j2 - junction ?d - junction ?u1 ?u2 - use)
            :precondition (and (togo ?c ?d)
                (allowed ?c ?r)
                (using ?r ?u1)
                (next ?u1 ?u2)
                (heavy ?r ?u2)
                (at ?c ?j1)
                (connected ?j1 ?r ?j2)
            )

    :effect (and (not (using ?r ?u1))
            (using ?r ?u2)
            (not (at ?c ?j1))
            (at ?c ?j2)
            (increase (total-cost) (length-heavy ?r))
        )
    )


    (:action DRIVE-TO-congested
        :parameters (?c - car ?j1 - junction ?r - road ?j2 - junction ?d - junction ?u1 - use)
            :precondition (and (togo ?c ?d)
                  (allowed ?c ?r)
                  (using ?r ?u1)
                  (cap ?r ?u1)
                  (at ?c ?j1)
                  (connected ?j1 ?r ?j2)
            )

    :effect (and (not (at ?c ?j1))
             (at ?c ?j2)
             (increase (total-cost) 100000)
         )
    )
)
