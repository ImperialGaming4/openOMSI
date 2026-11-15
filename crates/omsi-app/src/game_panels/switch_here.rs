//! The map camera (F4): a click on a vehicle asks whether to change into it, as in OMSI 2: another
//! vehicle standing in the world, or another cab of the train driven. A part nothing can be
//! driven from (a trailer, a tram's middle section) does not ask.

use super::{button, card, grow_row, note, title};
use omsi_plugin::ui::{Anchor, Panel};
use omsi_ui::theme::Palette;

/// The vehicle a click of the map camera asks to change into.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SwitchTo {
    /// The placed vehicle's `uid`; None: the vehicle driven now.
    vehicle: Option<u64>,
    /// The cab: the coupled part's index, None the leading vehicle's.
    cab: Option<usize>,
    name: String,
}

/// What a click on part `part` of a vehicle does.
#[derive(Debug, PartialEq)]
enum Answer {
    Ask,
    NotDrivable,
    AlreadyThere,
}

/// `seated`: the part the player sits at the wheel of in this vehicle, when it is the one driven
/// (None there: not at its wheel). `part` and `seated`'s value: None the leading vehicle.
fn answer(part: Option<usize>, seated: Option<Option<usize>>, drivable: bool) -> Answer {
    if seated == Some(part) {
        Answer::AlreadyThere
    } else if drivable {
        Answer::Ask
    } else {
        Answer::NotDrivable
    }
}

/// Whether part `part` of `v` can be driven from.
fn drivable(v: &omsi_sim::VehicleInstance, part: Option<usize>) -> bool {
    match part {
        None => !v.ty.def.cameras_driver.is_empty(),
        Some(k) => v.cab_seat(k).is_some(),
    }
}

fn name_of(v: &omsi_sim::VehicleInstance, part: Option<usize>) -> String {
    let def = part.and_then(|k| v.trailers.get(k)).map_or(&v.ty.def, |t| &t.ty.def);
    let stem = def.path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
    format!("{} {} ({stem})", def.manufacturer, def.type_name).trim().to_string()
}

pub(crate) fn panel(to: &SwitchTo, pal: Palette) -> Panel {
    let question = if to.vehicle.is_some() { "Change to this vehicle?" } else { "Change to this cab?" };
    let (yes, no) = (omsi_ui::tr("Yes"), omsi_ui::tr("Cancel"));
    card(Anchor::Center, 0.0, 0.0, 360.0, 236, pal, vec![title(&omsi_ui::tr(question)), note(&to.name, pal), grow_row(vec![button("yes", &yes, Some("check")), button("no", &no, Some("close"))])])
}

/// A click of the map camera that worked nothing of the bus: true when it landed on a vehicle
/// (the ground behind it is not what was clicked), and the question is up if there is one to ask.
pub(crate) fn ask(app: &mut crate::App) -> bool {
    let Some((o, d, _)) = app.cursor_ray_now() else { return false };
    let on_foot = app.session.on_foot.is_some();
    let driven = app.player.as_ref().and_then(|p| p.part_hit(o, d)).map(|(part, t)| (None, part, t));
    let placed = app.session.placed.iter().enumerate().filter_map(|(k, q)| q.part_hit(o, d).map(|(part, t)| (Some(k), part, t)));
    let Some((which, part, _)) = driven.into_iter().chain(placed).min_by(|a, b| a.2.total_cmp(&b.2)) else { return false };
    let (v, uid) = match which {
        Some(k) => (&app.session.placed[k].vehicle, Some(app.session.placed[k].uid)),
        None => match app.player.as_ref() {
            Some(p) => (&p.vehicle, None),
            None => return false,
        },
    };
    let seated = (uid.is_none() && !on_foot).then_some(v.active_cab);
    match answer(part, seated, drivable(v, part)) {
        Answer::Ask => {
            let to = SwitchTo { vehicle: uid, cab: part, name: name_of(v, part) };
            app.menus.game_panels.switch_to = Some(to);
        }
        Answer::NotDrivable => app.service_msg = Some((omsi_ui::tr("This part cannot be driven").into_owned(), 3.0)),
        Answer::AlreadyThere => {}
    }
    true
}

pub(crate) fn click(app: &mut crate::App, id: &str) {
    let Some(to) = app.menus.game_panels.switch_to.take() else { return };
    if id != "yes" {
        return;
    }
    match to.vehicle {
        Some(uid) => {
            // (gone meanwhile, the question with it)
            let Some(k) = app.session.placed.iter().position(|q| q.uid == uid) else { return };
            app.take_placed(k);
            if app.player.as_ref().is_none_or(|p| p.uid != uid) {
                return;
            }
        }
        None => app.back_to_bus(),
    }
    let Some(p) = app.player.as_mut() else { return };
    crate::on_foot::seat_in_cab(p, to.cab);
    app.game_action("view_set_driver");
    app.cam.look = (0.0, 0.0);
    app.sync_view_look();
    let message = omsi_ui::i18n::tr_fill("Now driving: {name}", &[("{name}", &to.name)]);
    app.service_msg = Some((message, 4.0));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_part_that_can_be_driven_asks() {
        assert_eq!(answer(Some(9), Some(None), true), Answer::Ask, "the other cab of the train driven");
        assert_eq!(answer(None, Some(Some(9)), true), Answer::Ask, "back to the leading vehicle's cab");
        assert_eq!(answer(None, None, true), Answer::Ask, "a placed vehicle, or the own one on foot");
    }

    #[test]
    fn a_part_that_cannot_be_driven_does_not_ask() {
        assert_eq!(answer(Some(3), Some(None), false), Answer::NotDrivable, "a section of the train driven");
        assert_eq!(answer(Some(0), None, false), Answer::NotDrivable, "the trailer of a placed vehicle");
    }

    #[test]
    fn the_cab_sat_in_is_no_change() {
        assert_eq!(answer(None, Some(None), true), Answer::AlreadyThere);
        assert_eq!(answer(Some(9), Some(Some(9)), true), Answer::AlreadyThere);
    }
}
