use abstutil::prettyprint_usize;
use map_gui::tools::ColorDiscrete;
use sim::Scenario;
use widgetry::mapspace::ToggleZoomed;
use widgetry::{
    Color, EventCtx, GfxCtx, HorizontalAlignment, Key, Line, Outcome, Panel, State, Text,
    VerticalAlignment, Widget,
};

use crate::app::{App, Transition};
use crate::common::CommonState;
use crate::devtools::destinations::PopularDestinations;

pub struct ScenarioManager {
    panel: Panel,
    scenario: Scenario,
    draw: ToggleZoomed,
}

impl ScenarioManager {
    pub fn new_state(scenario: Scenario, ctx: &mut EventCtx, app: &App) -> Box<dyn State<App>> {
        let mut colorer = ColorDiscrete::new(
            app,
            vec![
                ("1-2", Color::BLUE),
                ("3-4", Color::RED),
                ("more", Color::BLACK),
            ],
        );
        let mut total_cars_needed = 0;
        for (b, count) in scenario.count_parked_cars_per_bldg().consume() {
            total_cars_needed += count;
            let color = if count == 0 {
                continue;
            } else if count == 1 || count == 2 {
                "1-2"
            } else if count == 3 || count == 4 {
                "3-4"
            } else {
                "more"
            };
            colorer.add_b(b, color);
        }

        let (filled_spots, free_parking_spots) = app.primary.sim.get_all_parking_spots();
        assert!(filled_spots.is_empty());

        let (draw, legend) = colorer.build(ctx);
        Box::new(ScenarioManager {
            panel: Panel::new_builder(Widget::col(vec![
                Widget::row(vec![
                    Line(format!("Scenario {}", scenario.scenario_name))
                        .small_heading()
                        .into_widget(ctx),
                    ctx.style().btn_close_widget(ctx),
                ]),
                ctx.style()
                    .btn_outline
                    .text("popular destinations")
                    .hotkey(Key::D)
                    .build_def(ctx),
                Text::from_multiline(vec![
                    Line(format!(
                        "{} people",
                        prettyprint_usize(scenario.people.len())
                    )),
                    Line(format!(
                        "seed {} parked cars",
                        prettyprint_usize(total_cars_needed)
                    )),
                    Line(format!(
                        "{} parking spots",
                        prettyprint_usize(free_parking_spots.len()),
                    )),
                    Line(""),
                    Line("Parked cars per building"),
                ])
                .into_widget(ctx),
                legend,
            ]))
            .aligned(HorizontalAlignment::Right, VerticalAlignment::Top)
            .build(ctx),
            draw,
            scenario,
        })
    }
}

impl State<App> for ScenarioManager {
    fn event(&mut self, ctx: &mut EventCtx, app: &mut App) -> Transition {
        if let Outcome::Clicked(x) = self.panel.event(ctx) {
            match x.as_ref() {
                "close" => {
                    return Transition::Pop;
                }
                "popular destinations" => {
                    return Transition::Push(PopularDestinations::new_state(
                        ctx,
                        app,
                        &self.scenario,
                    ));
                }
                _ => unreachable!(),
            }
        }

        ctx.canvas_movement();
        if ctx.redo_mouseover() {
            app.recalculate_current_selection(ctx);
        }

        Transition::Keep
    }

    fn draw(&self, g: &mut GfxCtx, app: &App) {
        self.draw.draw(g);
        self.panel.draw(g);
        CommonState::draw_osd(g, app);
    }
}
