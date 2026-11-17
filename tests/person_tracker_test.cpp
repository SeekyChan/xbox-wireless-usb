#include "person_tracker.hpp"
#include <cassert>
#include <iostream>
#include <vector>

int main() {
    vision::PersonTracker tracker(2);
    vision::Detection a; a.box = vision::Rect(40, 40, 80, 180); a.score = 0.9f;
    vision::Detection b; b.box = vision::Rect(250, 55, 90, 170); b.score = 0.8f;
    tracker.update({a, b}, 640, 480);
    assert(tracker.tracks().size() == 2);
    assert(tracker.tracks()[0].id == 1);
    assert(tracker.tracks()[1].id == 2);

    // Small translation should keep the same IDs.
    a.box.x += 10; b.box.x -= 8;
    tracker.update({b, a}, 640, 480);
    assert(tracker.tracks().size() == 2);
    assert(tracker.tracks()[0].id == 1 && tracker.tracks()[0].hits == 2);
    assert(tracker.tracks()[1].id == 2 && tracker.tracks()[1].hits == 2);

    // Temporary absence: retain the stale track but mark it unmatched.
    tracker.update({a}, 640, 480);
    assert(tracker.tracks().size() == 2);
    assert(!tracker.tracks()[1].matched);
    tracker.update({a}, 640, 480);
    assert(tracker.tracks().size() == 2);
    tracker.update({a}, 640, 480);
    assert(tracker.tracks().size() == 1);
    assert(tracker.tracks()[0].id == 1);

    // A new person receives a new ID (not a biometric identity).
    vision::Detection c; c.box = vision::Rect(420, 40, 70, 150); c.score = 0.75f;
    tracker.update({a, c}, 640, 480);
    assert(tracker.tracks().size() == 2);
    assert(tracker.tracks()[1].id == 3);

    tracker.reset();
    assert(tracker.tracks().empty());
    std::cout << "person_tracker_test PASS\n";
    return 0;
}
