use super::*;

/// Create a traffic signal which has a stage that is: protected straight, protected right,
/// unprotected left, unprotected right on red. Followed by a variable stage that has protected
/// left, unprotected right on red. With a last stage that is all-walk and variable.
/// In some degenerate cases, usually with one or more one-way, this can reduce to stage per road.
/// In some rare cases, usually with an alleyway, oncoming lanes can't both be protected left turns.
/// In such cases the stage is split into two stages with each having a protected and yeild turn.
pub fn make_traffic_signal(map: &Map, i: &Intersection) -> Option<ControlTrafficSignal> {
    // Try to create the stages, this returns a unoptimized signal, which is then optimized.
    if let Some(ts) = make_signal(i, map) {
        return optimize(ts, i);
    }
    None
}

fn make_signal(i: &Intersection, map: &Map) -> Option<ControlTrafficSignal> {
    let mut ts = new(i.id);
    if let Some(other) = three_way_three_stage(i, map) {
        ts.stages = other.stages;
    } else if let Some(other) = four_way_four_stage(i, map) {
        ts.stages = other.stages;
    }
    ts.convert_to_ped_scramble_without_promotion(i);
    // We don't always get a valid traffic signal from the default 3-way and 4-way. When we don't
    // we need to try assembling stages with a more complex algorithm.
    if ts.validate(i).is_err() {
        if let Some(other) = multi_way_stages(i) {
            ts.stages = other.stages;
            ts.convert_to_ped_scramble_without_promotion(i);
        }
    }
    if let Err(err) = ts.validate(i) {
        // when all else fails, use stage per road and all-walk stage at the end
        debug!("multi-way validation_error={} ts={:#?}", err, ts);
        ts = stage_per_road(map, i);
        ts.convert_to_ped_scramble(i);
    }
    Some(ts)
}

fn optimize(mut ts: ControlTrafficSignal, i: &Intersection) -> Option<ControlTrafficSignal> {
    // Remove stages which don't contain a protected route.
    ts.stages.retain(|s| !s.protected_movements.is_empty());
    // Determine if any stages can be merged. We could merge turns, but if we end up not reducing
    // the stage as a result, its probably not worth doing, or can be easily added by the user.
    while let Some(merged_ts) = merge_stages(&ts, i) {
        ts = merged_ts;
    }
    make_lagging_green_variable(&mut ts);
    make_crosswalk_variable(&mut ts, i);
    Some(ts)
}

// convert walk to variable with a min duration not less than 15 seconds
fn make_crosswalk_variable(ts: &mut ControlTrafficSignal, i: &Intersection) {
    const MIN_CROSSWALK_TIME: Duration = Duration::const_seconds(15.0);
    for s in &mut ts.stages {
        if let Some(duration) = s.max_crosswalk_time(i) {
            if let StageType::Fixed(_) = s.stage_type {
                s.stage_type = StageType::Variable(
                    duration.max(MIN_CROSSWALK_TIME),
                    Duration::const_seconds(1.0),
                    Duration::const_seconds(1.0),
                )
            }
        }
    }
}

fn merge_stages(ts: &ControlTrafficSignal, i: &Intersection) -> Option<ControlTrafficSignal> {
    for s_src in &ts.stages {
        // s_src is the stage we want to apply to the other stages
        for s_dst in &ts.stages {
            if s_src == s_dst {
                continue;
            }
            let mut merged_stage = s_dst.clone();
            merged_stage
                .protected_movements
                .extend(s_src.protected_movements.clone());

            let mut maybe_ts = ts.clone();
            // insert at the head, keeping crosswalk last
            maybe_ts.stages.insert(0, merged_stage);
            if maybe_ts.validate(i).is_ok() {
                let mut stages: Vec<Stage> = Vec::new();
                for s in maybe_ts.stages {
                    if s != *s_src && s != *s_dst {
                        stages.push(s);
                    }
                }
                maybe_ts.stages = stages;
                return Some(maybe_ts);
            }
        }
    }
    None
}

// Sometimes protected oncoming left turns aren't possible.
fn is_conflict(stage: &Stage, i: &Intersection) -> Option<(MovementID, MovementID)> {
    for m1 in stage.protected_movements.iter().map(|m| &i.movements[m]) {
        for m2 in stage.protected_movements.iter().map(|m| &i.movements[m]) {
            // Use low-level turn conflict, since we know this a road to road movement.
            if m1.id != m2.id && m1.geom.intersection(&m2.geom).is_some() {
                return Some((m1.id, m2.id));
            }
        }
    }
    None
}

fn protected_yield_stage(p: MovementID, y: MovementID) -> Stage {
    let mut stage = Stage::new();
    stage.protected_movements.insert(p);
    stage.yield_movements.insert(y);
    stage
}

/// Build stages. First find roads that are straight across, they are either one-way or two-way.
/// For one-way, add any right or left turns, thus completing the stage. For two-way, two
/// stages will be added. The first stage has protected straight, and right and yield left.
/// The second stage has protected left. Lastly, sometimes oncomming left turns can't both
/// be protected, if this occurs the 2nd stage will have one direction protected and the
/// other yeild and a 3rd, inverse, stage will be added which has the other direction's left
/// protected and other yield. Finally, any turns which weren't assigned, because there
/// are no straights or there are more than just pairs of straight intersections, are assigned a
/// stage. These, too are handled as pairs until one remains, which is handled as a one-way.
fn multi_way_stages(i: &Intersection) -> Option<ControlTrafficSignal> {
    let mut ts = new(i.id);
    let (mut right, mut left, straight, mut roads) = movements(i);
    let (one_way, two_way) = straight_types(&straight);
    for m in &one_way {
        let mut stage = Stage::new();
        stage.protected_movements.insert(*m);
        for t in movements_from(m.from.road, &right) {
            stage.protected_movements.insert(t);
        }
        for t in movements_from(m.from.road, &left) {
            stage.protected_movements.insert(t);
        }
        add_stage(&mut ts, stage);
        roads.remove(&m.from.road);
    }
    for (m1, m2) in &two_way {
        let mut stage1 = Stage::new();
        let mut stage2 = Stage::new();
        // Insert the straight movements, followed by the right and then the left.
        stage1.protected_movements.insert(*m1);
        stage1.protected_movements.insert(*m2);
        stage1
            .protected_movements
            .extend(movements_from(m1.from.road, &right));
        stage1
            .protected_movements
            .extend(movements_from(m2.from.road, &right));
        for t in movements_from(m1.from.road, &left) {
            stage1.yield_movements.insert(t);
            stage2.protected_movements.insert(t);
        }
        for t in movements_from(m2.from.road, &left) {
            stage1.yield_movements.insert(t);
            stage2.protected_movements.insert(t);
        }
        add_stage(&mut ts, stage1);
        if let Some((m1, m2)) = is_conflict(&stage2, i) {
            // We've hit the case where oncoming left turns can't both be protected.
            add_stage(&mut ts, protected_yield_stage(m1, m2));
            add_stage(&mut ts, protected_yield_stage(m2, m1));
        } else {
            add_stage(&mut ts, stage2);
        }
        roads.remove(&m1.from.road);
        roads.remove(&m2.from.road);
    }
    // We may be done assigning, or we may have some roads we haven't dealt with yet.
    let mut vec: Vec<_> = roads.into_iter().collect();
    // We're going to treat the roads as if thery are straight. Otherwise,
    // we'd end up with overlapping protected left turns.
    while let Some(r1) = vec.pop() {
        let mut stage1 = Stage::new();
        let mut stage2 = Stage::new();
        if let Some(r2) = vec.pop() {
            // dual stage, with lagging left turns
            if let Some(m) = remove_movement(r1, r2, &mut right) {
                stage1.protected_movements.insert(m);
            } else if let Some(m) = remove_movement(r1, r2, &mut left) {
                stage1.protected_movements.insert(m);
            }
            if let Some(m) = remove_movement(r2, r1, &mut right) {
                stage1.protected_movements.insert(m);
            } else if let Some(m) = remove_movement(r2, r1, &mut left) {
                stage1.protected_movements.insert(m);
            }

            // add right turns
            stage1
                .protected_movements
                .extend(movements_from(r1, &right));
            stage1
                .protected_movements
                .extend(movements_from(r2, &right));

            // add left turns
            for t in movements_from(r1, &left) {
                stage1.yield_movements.insert(t);
                stage2.protected_movements.insert(t);
            }
            for t in movements_from(r2, &left) {
                stage1.yield_movements.insert(t);
                stage2.protected_movements.insert(t);
            }
            // add the stages
            add_stage(&mut ts, stage1);
            add_stage(&mut ts, stage2);
        } else {
            // single stage without lagging left turns
            stage1
                .protected_movements
                .extend(movements_from(r1, &right));
            stage1.protected_movements.extend(movements_from(r1, &left));
            add_stage(&mut ts, stage1);
        }
    }
    Some(ts)
}

fn add_stage(ts: &mut ControlTrafficSignal, stage: Stage) {
    // If there aren't any protected movements, don't add it.
    if stage.protected_movements.is_empty() {
        return;
    }
    // Ensure a duplicate isn't added.
    if ts.stages.iter().all(|s| *s != stage) {
        ts.stages.push(stage)
    }
}

fn movements_from(from: RoadID, movements: &[MovementID]) -> Vec<MovementID> {
    movements
        .iter()
        .filter_map(|mvmnt| {
            if from == mvmnt.from.road {
                Some(*mvmnt)
            } else {
                None
            }
        })
        .collect()
}

fn remove_movement(
    from: RoadID,
    to: RoadID,
    movements: &mut Vec<MovementID>,
) -> Option<MovementID> {
    let idx = movements
        .iter()
        .position(|mvmnt| mvmnt.from.road == from && mvmnt.to.road == to)?;
    Some(movements.remove(idx))
}

fn three_way_three_stage(i: &Intersection, map: &Map) -> Option<ControlTrafficSignal> {
    let roads = i.get_sorted_incoming_roads(map);
    if roads.len() != 3 {
        return None;
    }
    let mut ts = new(i.id);

    // Picture a T intersection. Use turn angles to figure out the "main" two roads.
    let straight = i
        .movements
        .values()
        .find(|g| g.turn_type == TurnType::Straight)?;
    let (north, south) = (straight.id.from.road, straight.id.to.road);
    let east = roads
        .into_iter()
        .find(|r| *r != north && *r != south)
        .unwrap();

    // Three-stage with protected lefts, right turn on red
    make_stages(
        &mut ts,
        &map.config,
        i,
        vec![
            vec![
                (vec![north, south], TurnType::Straight, PROTECTED),
                (vec![north, south], TurnType::Right, PROTECTED),
                (vec![north, south], TurnType::Left, YIELD),
                (vec![east], TurnType::Right, YIELD),
            ],
            vec![
                (vec![north, south], TurnType::Left, PROTECTED),
                (vec![east], TurnType::Right, YIELD),
            ],
            vec![
                (vec![east], TurnType::Straight, PROTECTED),
                (vec![east], TurnType::Right, PROTECTED),
                (vec![east], TurnType::Left, PROTECTED),
                (vec![north, south], TurnType::Right, YIELD),
            ],
        ],
    );
    Some(ts)
}

fn four_way_four_stage(i: &Intersection, map: &Map) -> Option<ControlTrafficSignal> {
    let roads = i.get_sorted_incoming_roads(map);
    if roads.len() != 4 {
        return None;
    }

    // Just to refer to these easily, label with directions. Imagine an axis-aligned four-way.
    let (north, west, south, east) = (roads[0], roads[1], roads[2], roads[3]);

    // Four-stage with protected lefts, right turn on red (except for the protected lefts),
    // turning cars yield to peds
    let mut ts = new(i.id);
    make_stages(
        &mut ts,
        &map.config,
        i,
        vec![
            vec![
                (vec![north, south], TurnType::Straight, PROTECTED),
                (vec![north, south], TurnType::Left, YIELD),
                (vec![north, south], TurnType::Right, PROTECTED),
                (vec![east, west], TurnType::Right, YIELD),
            ],
            vec![
                (vec![north, south], TurnType::Left, PROTECTED),
                (vec![east, west], TurnType::Right, YIELD),
            ],
            vec![
                (vec![east, west], TurnType::Straight, PROTECTED),
                (vec![east, west], TurnType::Left, YIELD),
                (vec![east, west], TurnType::Right, PROTECTED),
                (vec![north, south], TurnType::Right, YIELD),
            ],
            vec![
                (vec![east, west], TurnType::Left, PROTECTED),
                (vec![north, south], TurnType::Right, YIELD),
            ],
        ],
    );
    Some(ts)
}

fn movements(
    i: &Intersection,
) -> (
    Vec<MovementID>,
    Vec<MovementID>,
    Vec<MovementID>,
    BTreeSet<RoadID>,
) {
    let mut right: Vec<MovementID> = Vec::new();
    let mut left: Vec<MovementID> = Vec::new();
    let mut straight: Vec<MovementID> = Vec::new();
    let mut set: BTreeSet<RoadID> = BTreeSet::new();

    for (id, m) in &i.movements {
        if !id.crosswalk {
            match m.turn_type {
                TurnType::Right => right.push(*id),
                TurnType::Left => left.push(*id),
                TurnType::Straight => straight.push(*id),
                _ => (),
            }
            set.insert(id.from.road);
        }
    }
    (right, left, straight, set)
}

fn straight_types(movements: &[MovementID]) -> (Vec<MovementID>, Vec<(MovementID, MovementID)>) {
    let mut one_way: Vec<MovementID> = Vec::new();
    let mut two_way: Vec<(MovementID, MovementID)> = Vec::new();
    for m in movements {
        if let Some(other) = movements
            .iter()
            .find(|&other| m.from.road == other.to.road && m.to.road == other.from.road)
        {
            two_way.push((*m, *other));
        } else {
            one_way.push(*m);
        }
    }
    (one_way, two_way)
}

fn make_lagging_green_variable(ts: &mut ControlTrafficSignal) {
    const EXTENT_DURATION: Duration = Duration::const_seconds(10.0);
    const MAX_DURATION: Duration = Duration::const_seconds(20.0);
    let mut prev_stage: Option<&mut Stage> = None;
    for stage in ts.stages.iter_mut() {
        // Lagging green: if this stage's protected movements were yield movements in the
        // previous stage, make this stage optional.
        if let Some(prev) = prev_stage {
            if stage
                .protected_movements
                .iter()
                .all(|m| prev.yield_movements.contains(m))
            {
                if let StageType::Fixed(_) = stage.stage_type {
                    stage.stage_type =
                        StageType::Variable(Duration::ZERO, EXTENT_DURATION, MAX_DURATION);
                }
            }
        }
        prev_stage = Some(stage);
    }
}
