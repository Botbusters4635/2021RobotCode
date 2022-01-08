//
// Created by abiel on 9/29/21.
//

#include "ChecklistItem.h"

ChecklistItem::ChecklistItem(const std::string &name) : Module(name) {
	;
}

void ChecklistItem::assertTest(const std::string &testName, bool result) {
	expectTest(testName, result);
	if (!result) {
		log->error("Halting {} checklist", testName);
		Cancel();
	}
}

void ChecklistItem::expectTest(const std::string &testName, bool result) {
	if (result) {
		log->info("{}/\"{}\" passed", getName(), testName);
	} else {
		log->error("{}/\"{}\" failed!", getName(), testName);
	}
}