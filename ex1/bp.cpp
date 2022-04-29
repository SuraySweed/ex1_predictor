/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */
#include <iostream>
#include <math.h>
#include <vector>
#include <stdint.h>
#include <stdbool.h>
#include "bp_api.h"

using std::vector;

typedef struct {
	uint32_t tag;
	uint32_t target;
} btbEntry_t;

typedef struct {
	bool branch;
	uint32_t target;
} prediction_t;

enum SHARE_STATE {
	NO_SHARE = 0,
	LSB_SHARE = 1,
	MID_SHARE = 2,
};

enum FSM_PREDICTOIN {
	SNT = 0,
	WNT = 1,
	WT = 2,
	ST = 3,
};

class BranchPredictor {
private:
	FSM_PREDICTOIN fsm_state;
	SHARE_STATE share_state;
	SIM_stats stats;
	vector<vector<FSM_PREDICTOIN>> fsm_vector;
	vector<uint32_t> history_vector;
	vector<btbEntry_t> btb_vector;
	bool is_global_history;
	bool is_global_table;
	uint32_t history_size;
	uint32_t btb_size;
	uint32_t tag_size;

	void updatePrediction(FSM_PREDICTOIN& fsm, bool isUpdate);
	uint32_t getSharePolicy(uint32_t pc, uint32_t hist);
	unsigned getTotalSize(unsigned btb_size, unsigned hist_size, unsigned tag_size, bool is_global_hist, bool is_global_table);
	uint32_t getBtbIndex(uint32_t pc);
	uint32_t getTag(uint32_t pc);

public:
	BranchPredictor(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
		bool isGlobalHist, bool isGlobalTable, int Shared);
	~BranchPredictor() = default;
	BranchPredictor(BranchPredictor& other) = default;
	prediction_t predict(uint32_t pc);
	void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);
	SIM_stats getStats();
};


void BranchPredictor::updatePrediction(FSM_PREDICTOIN& fsm, bool isUpdate)
{
	switch (fsm)
	{
	case SNT:
		fsm = (isUpdate) ? WNT : SNT;
		break;

	case WNT:
		fsm = (isUpdate) ? WT : SNT;
		break;

	case WT:
		fsm = (isUpdate) ? ST : WNT;
		break;

	case ST:
		fsm = (isUpdate) ? ST : WT;
		break;

	default:
		break;
	}
}

uint32_t BranchPredictor::getSharePolicy(uint32_t pc, uint32_t hist)
{
	hist = hist & history_size;
	pc = pc >> 2;

	if (share_state == NO_SHARE) {
		return hist;
	}
	else if (share_state == LSB_SHARE) {
		return (hist ^ pc) & history_size;
	}
	else if (share_state == MID_SHARE) {
		pc = pc >> 14; //start in the 16 bit
		return (hist ^ pc) & history_size;
	}
	else {
		return hist;
	}
}

unsigned BranchPredictor::getTotalSize(unsigned btb_size, unsigned hist_size, unsigned tag_size,
	bool is_global_hist, bool is_global_table)
{
	unsigned size = btb_size * (tag_size + 30); // 30 is address bit size
	if (is_global_history) {
		size += hist_size;
	}
	else {
		size += (btb_size * hist_size);
	}

	if (is_global_table) {
		size += (2 * pow(2, hist_size));
	}
	else {
		size += (btb_size * 2 * pow(2, hist_size));
	}

	return size;
}

uint32_t BranchPredictor::getBtbIndex(uint32_t pc)
{
	int btb_indexMask = (int)log2(btb_size);
	pc >>= 2;
	return (pc & (uint32_t)(pow(2, btb_indexMask) - 1));
}

uint32_t BranchPredictor::getTag(uint32_t pc)
{
	pc >>= (2 + (int)log2(btb_size));
	return (pc & ((uint32_t)pow(2, tag_size) - 1));
}

BranchPredictor::BranchPredictor(unsigned btbSize, unsigned historySize, unsigned tagSize,
	unsigned fsmState, bool isGlobalHist, bool isGlobalTable, int Shared) : fsm_state(static_cast<FSM_PREDICTOIN>(fsmState)),
	share_state(static_cast<SHARE_STATE>(Shared)), stats({ 0, 0, 0 }),
	is_global_history(isGlobalHist), is_global_table(isGlobalTable), btb_size(btbSize), tag_size(tagSize)
{
	history_size = pow(2, historySize) - 1;
	stats.size = getTotalSize(btbSize, historySize, tagSize, isGlobalHist, isGlobalTable);
	//we have check the situation (hist_size <= max_hist_size)
	uint32_t numOfFSM;
	if (isGlobalTable) 	numOfFSM = 1;
	else numOfFSM = btbSize;
	fsm_vector = vector<vector<FSM_PREDICTOIN>>(numOfFSM, vector<FSM_PREDICTOIN>(pow(2, historySize), fsm_state));

	if (is_global_history) {
		history_vector = vector<uint32_t>(1, 0);
	}
	else {
		history_vector = vector<uint32_t>(btbSize, 0);
	}

	btb_vector = vector<btbEntry_t>(btbSize, { INT_MAX, 0 });
}

prediction_t BranchPredictor::predict(uint32_t pc)
{
	prediction_t prediction_result = { false, pc + 4 };
	uint32_t tag = getTag(pc);
	uint32_t index = getBtbIndex(pc);
	uint32_t fsm_table_index = (is_global_table) ? 0 : index;
	uint32_t history_index = (is_global_history) ? 0 : index;

	if (btb_vector[index].tag != tag) {
		return prediction_result;
	}

	uint32_t history = history_vector[history_index];
	FSM_PREDICTOIN pred = fsm_vector[fsm_table_index][getSharePolicy(pc, history)];
	if (pred == SNT || pred == WNT) {
		prediction_result.branch = false;
	}
	else {
		prediction_result.branch = true;
		prediction_result.target = btb_vector[index].target;
	}

	return prediction_result;
}

void BranchPredictor::update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst)
{
	stats.br_num++;
	prediction_t predicted_taken = predict(pc);

	if ((taken && predicted_taken.target != targetPc) ||
		(!taken && predicted_taken.target != pc + 4)) {
		stats.flush_num++;
	}

	uint32_t tag = getTag(pc);
	uint32_t index = getBtbIndex(pc);
	uint32_t fsm_table_index = (is_global_table) ? 0 : index;
	uint32_t history_index = (is_global_history) ? 0 : index;
	uint32_t& history = history_vector[history_index];

	if (btb_vector[index].tag != tag || btb_vector[index].tag == INT_MAX) {
		btb_vector[index] = { tag, targetPc };
		if (!is_global_history) {
			history = 0;
		}
		if (!is_global_table) {
			fsm_vector[fsm_table_index] = vector<FSM_PREDICTOIN>(fsm_vector[fsm_table_index].size(), fsm_state);
		}
	}

	btb_vector[index].target = targetPc;

	FSM_PREDICTOIN& prediction = fsm_vector[fsm_table_index][getSharePolicy(pc, history)];
	history <<= 1;
	history += taken;
	history = history & history_size;
	updatePrediction(prediction, taken);
}

SIM_stats BranchPredictor::getStats()
{
	return stats;
}



BranchPredictor bp(0, 0, 0, 0, 0, 0, 0);

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	bp = BranchPredictor(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	prediction_t result = bp.predict(pc);
	*dst = result.target;
	return result.branch;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	bp.update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
	*curStats = bp.getStats();
}

