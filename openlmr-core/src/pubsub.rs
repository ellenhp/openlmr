// Reminder: Event buses can be an anti-pattern when event handlers emit additional events. Please avoid this.
pub const EVENT_CAP: usize = 64;
pub const EVENT_PUBS: usize = 8;
pub const EVENT_SUBS: usize = 8;
