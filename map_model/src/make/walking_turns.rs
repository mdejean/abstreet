use geom::{Distance, Line, PolyLine, Pt2D, Ring};

use crate::{
    Direction, DrivingSide, Intersection, IntersectionID, Lane, LaneID, LaneType, Map, Turn,
    TurnID, TurnType,
};

/// Generate Crosswalk and SharedSidewalkCorner (places where two sidewalks directly meet) turns.
/// UnmarkedCrossings are not generated here; another process later "downgrades" crosswalks to
/// unmarked.
/// A complete rewrite of make_walking_turns, which looks at all sidewalks (or lack thereof) in
/// counter-clockwise order around an intersection. Based on adjacency, create a
/// SharedSidewalkCorner or a Crosswalk.
pub fn make_walking_turns(map: &Map, i: &Intersection) -> Vec<Turn> {
    let driving_side = map.config.driving_side;

    // Consider all roads in counter-clockwise order. Every road has up to two sidewalks. Gather
    // those in order, remembering what roads don't have them.
    let mut lanes: Vec<Option<&Lane>> = Vec::new();
    let mut num_sidewalks = 0;
    let mut sorted_roads = i.get_roads_sorted_by_incoming_angle(map);
    // And for left-handed driving, we need to walk around in the opposite order.
    if driving_side == DrivingSide::Left {
        sorted_roads.reverse();
    }

    for r in sorted_roads {
        let road = map.get_r(r);
        let mut fwd = None;
        let mut back = None;
        for l in &road.lanes {
            if l.lane_type.is_walkable() {
                if l.dir == Direction::Fwd {
                    fwd = Some(l);
                } else {
                    back = Some(l);
                }
            }
        }
        if fwd.is_some() {
            num_sidewalks += 1;
        }
        if back.is_some() {
            num_sidewalks += 1;
        }
        let (in_lane, out_lane) = if road.src_i == i.id {
            (back, fwd)
        } else {
            (fwd, back)
        };
        lanes.push(in_lane);
        lanes.push(out_lane);
    }
    if num_sidewalks <= 1 {
        return Vec::new();
    }
    // Make sure we start with a sidewalk.
    while lanes[0].is_none() {
        lanes.rotate_left(1);
    }
    let mut result: Vec<Turn> = Vec::new();

    let mut from: Option<&Lane> = lanes[0];
    let first_from = from.unwrap().id;
    let mut adj = true;
    for l in lanes.iter().skip(1).chain(lanes.iter()) {
        if i.id.0 == 284 {
            debug!(
                "looking at {:?}. from is {:?}, first_from is {}, adj is {}",
                l.map(|l| l.id),
                from.map(|l| l.id),
                first_from,
                adj
            );
        }

        if from.is_none() {
            from = *l;
            adj = true;
            continue;
        }
        let l1 = from.unwrap();

        if l.is_none() {
            adj = false;
            continue;
        }
        let l2 = l.unwrap();

        if adj && l1.id.road != l2.id.road {
            let geom = make_shared_sidewalk_corner(driving_side, i, l1, l2);
            result.push(Turn {
                id: turn_id(i.id, l1.id, l2.id),
                turn_type: TurnType::SharedSidewalkCorner,
                geom,
            });

            from = Some(l2);
        // adj stays true
        } else {
            // Only make one crosswalk for degenerate intersections
            if !i.is_degenerate() || !result.iter().any(|t| t.turn_type == TurnType::Crosswalk) {
                result.extend(
                    make_crosswalks(i, l1, l2, driving_side)
                        .into_iter()
                        .flatten(),
                );
            }
            from = Some(l2);
            adj = true;
        }

        // Have we made it all the way around?
        if first_from == from.unwrap().id {
            break;
        }
    }

    result
}

/// Filter out crosswalks on really short roads. In reality, these roads are usually located within
/// an intersection, which isn't a valid place for a pedestrian crossing.
///
/// And if the road is marked as having no crosswalks at an end, downgrade them to unmarked
/// crossings.
pub fn filter_turns(mut input: Vec<Turn>, map: &Map, i: &Intersection) -> Vec<Turn> {
    for r in &i.roads {
        if map.get_r(*r).is_extremely_short() {
            input.retain(|t| {
                !(t.id.src.road == *r && t.id.dst.road == *r && t.turn_type.pedestrian_crossing())
            });
        }
    }

    for turn in &mut input {
        if let Some(dr) = turn.crosswalk_over_road(map) {
            let road = map.get_r(dr.road);
            let keep = if dr.dir == Direction::Fwd {
                road.crosswalk_forward
            } else {
                road.crosswalk_backward
            };
            if !keep {
                turn.turn_type = TurnType::UnmarkedCrossing;
            }
        } else if turn.turn_type.pedestrian_crossing() {
            // We have a crosswalk over multiple roads (or sometimes, just one road that only has a
            // walkable lane on one side of it). We can't yet detect all the roads crossed. So for
            // now, it's more often correct to assume that if any nearby roads don't have a
            // crossing snapped to both ends, then there's probably no crosswalk here.
            for l in [turn.id.src, turn.id.dst] {
                let road = map.get_parent(l);
                if !road.crosswalk_forward || !road.crosswalk_backward {
                    turn.turn_type = TurnType::UnmarkedCrossing;
                }
            }
        }
    }

    input
}

/// At an intersection of footpaths only, just generate a turn between every pair of lanes.
fn make_footway_turns(map: &Map, i: &Intersection) -> Vec<Turn> {
    let lanes = i
        .incoming_lanes
        .iter()
        .chain(&i.outgoing_lanes)
        .filter_map(|l| {
            let l = map.get_l(*l);
            if l.is_walkable() {
                Some(l)
            } else {
                None
            }
        })
        .collect::<Vec<&Lane>>();
    let mut results = Vec::new();
    for l1 in &lanes {
        for l2 in &lanes {
            // Only generate one turn for each pair
            if l1.id >= l2.id {
                continue;
            }
            let maybe_geom = PolyLine::new(vec![l1.endpoint(i.id), l2.endpoint(i.id)]);
            let geom = maybe_geom.unwrap_or_else(|_| {
                // TODO Gross! After improving intersection geometry where these cases are
                // happening, if this still happens, maybe it's time to make turn geometry be
                // optional.
                PolyLine::must_new(vec![l1.endpoint(i.id), l1.endpoint(i.id).offset(0.1, 0.1)])
            });
            results.push(Turn {
                id: turn_id(i.id, l1.id, l2.id),
                turn_type: TurnType::SharedSidewalkCorner,
                geom,
            });
        }
    }
    results
}

fn make_crosswalks(
    i: &Intersection,
    l1: &Lane,
    l2: &Lane,
    driving_side: DrivingSide,
) -> Option<Vec<Turn>> {
    let l1_line = l1.end_line(i.id);
    let l2_line = l2.end_line(i.id);

    // Jut out a bit into the intersection, cross over, then jut back in.
    // Put degenerate intersection crosswalks in the middle (DEGENERATE_HALF_LENGTH).
    let geom_fwds = PolyLine::deduping_new(vec![
        l1_line.pt2(),
        l1_line.unbounded_dist_along(
            l1_line.length()
                + if i.is_degenerate() {
                    Distance::const_meters(2.5)
                } else {
                    l1.width / 2.0
                },
        ),
        l2_line.unbounded_dist_along(
            l2_line.length()
                + if i.is_degenerate() {
                    Distance::const_meters(2.5)
                } else {
                    l2.width / 2.0
                },
        ),
        l2_line.pt2(),
    ])
    .ok()?;

    Some(vec![Turn {
        id: turn_id(i.id, l1.id, l2.id),
        turn_type: TurnType::Crosswalk,
        geom: geom_fwds,
    }])
}

// TODO This doesn't handle sidewalk/shoulder transitions
fn make_shared_sidewalk_corner(
    driving_side: DrivingSide,
    i: &Intersection,
    l1: &Lane,
    l2: &Lane,
) -> PolyLine {
    let (start_pt, end_pt) = (l1.endpoint(i.id), l2.endpoint(i.id));

    if let Some(offset_p) = i.polygon.clone().offset(-l1.width.min(l2.width) / 2.0, 1.0) {
        if let Ok(pl) = PolyLine::new(offset_p.into_points()) {
            // Find all of the points on the intersection polygon between the two sidewalks.
            let (p1, p2) = (pl.nearest_pt(start_pt), pl.nearest_pt(end_pt));

            if let Ok(r) = Ring::new(pl.into_points()) {
                if let Some(pl) = r.get_shorter_slice_between(p1, p2) {
                    return pl;
                }
            }
        }
    }
    warn!(
        "SharedSidewalkCorner between {} and {} has weird duplicate geometry, so just \
            doing straight line",
        l1.id, l2.id
    );
    return PolyLine::must_new(vec![start_pt, end_pt]);
}

fn turn_id(parent: IntersectionID, src: LaneID, dst: LaneID) -> TurnID {
    TurnID { parent, src, dst }
}
