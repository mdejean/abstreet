use crate::objects::DrawCtx;
use crate::plugins::{NonblockingPlugin, PluginCtx};
use ezgui::{Color, GfxCtx, Key};
use geom::{Duration, Line};
use map_model::{Trace, LANE_THICKNESS};
use sim::TripID;

pub struct DiffTripState {
    time: Duration,
    trip: TripID,
    // These are all optional because mode-changes might cause temporary interruptions.
    // Just point from primary world agent to secondary world agent.
    line: Option<Line>,
    primary_route: Option<Trace>,
    secondary_route: Option<Trace>,
}

impl DiffTripState {
    pub fn new(ctx: &mut PluginCtx) -> Option<DiffTripState> {
        if let Some(agent) = ctx.primary.current_selection.and_then(|id| id.agent_id()) {
            if let Some(trip) = ctx.primary.sim.agent_to_trip(agent) {
                if ctx
                    .input
                    .contextual_action(Key::B, &format!("Show {}'s parallel world", agent))
                {
                    return Some(diff_trip(trip, ctx));
                }
            }
        }
        None
    }
}

impl NonblockingPlugin for DiffTripState {
    fn nonblocking_event(&mut self, ctx: &mut PluginCtx) -> bool {
        ctx.input.set_mode_with_prompt(
            "A/B Trip Explorer",
            format!("A/B Trip Explorer for {}", self.trip),
            &ctx.canvas,
        );
        if ctx.input.modal_action("auit") {
            return false;
        } else if self.time != ctx.primary.sim.time() {
            *self = diff_trip(self.trip, ctx);
        }
        true
    }

    fn draw(&self, g: &mut GfxCtx, ctx: &DrawCtx) {
        if let Some(l) = &self.line {
            g.draw_line(
                ctx.cs.get_def("diff agents line", Color::YELLOW),
                LANE_THICKNESS,
                l,
            );
        }
        if let Some(t) = &self.primary_route {
            g.draw_polygon(
                ctx.cs.get_def("primary agent route", Color::RED.alpha(0.5)),
                &t.make_polygons(LANE_THICKNESS),
            );
        }
        if let Some(t) = &self.secondary_route {
            g.draw_polygon(
                ctx.cs
                    .get_def("secondary agent route", Color::BLUE.alpha(0.5)),
                &t.make_polygons(LANE_THICKNESS),
            );
        }
    }
}

fn diff_trip(trip: TripID, ctx: &mut PluginCtx) -> DiffTripState {
    let primary_sim = &ctx.primary.sim;
    let primary_map = &ctx.primary.map;
    let (secondary_sim, secondary_map) = ctx
        .secondary
        .as_ref()
        .map(|(s, _)| (&s.sim, &s.map))
        .unwrap();

    let pt1 = primary_sim.get_canonical_pt_per_trip(trip, primary_map);
    let pt2 = secondary_sim.get_canonical_pt_per_trip(trip, secondary_map);
    let line = if pt1.is_some() && pt2.is_some() {
        Line::maybe_new(pt1.unwrap(), pt2.unwrap())
    } else {
        None
    };
    let primary_route = primary_sim
        .trip_to_agent(trip)
        .and_then(|agent| primary_sim.trace_route(agent, primary_map, None));
    let secondary_route = secondary_sim
        .trip_to_agent(trip)
        .and_then(|agent| secondary_sim.trace_route(agent, secondary_map, None));

    if line.is_none() || primary_route.is_none() || secondary_route.is_none() {
        println!("{} isn't present in both sims", trip);
    }
    DiffTripState {
        time: primary_sim.time(),
        trip,
        line,
        primary_route,
        secondary_route,
    }
}
