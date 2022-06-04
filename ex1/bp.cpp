/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */
#include <iostream>
#include <math.h>
#include <vector>
#include <stdint.h>
#include <stdbool.h>
#include "bp_api.h"

#define INT_MAX 2147483647

using std::vector;


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
	vector<uint32_t> btb_targets;
	vector<uint32_t> btb_tags;
	bool is_global_history;
	bool is_global_table;
	uint32_t history_size;
	uint32_t btb_size;
	uint32_t tag_size;

	void updatePrediction(FSM_PREDICTOIN& fsm, bool isUpdate);
	uint32_t getDataBySharePolicy(uint32_t pc, uint32_t hist);
	unsigned getTotalSize(unsigned btb_size, unsigned hist_size, unsigned tag_size, bool is_global_hist, bool is_global_table);
	uint32_t getBtbIndex(uint32_t pc);
	uint32_t getTag(uint32_t pc);

public:
	BranchPredictor(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
		bool isGlobalHist, bool isGlobalTable, int Shared);
	~BranchPredictor() = default;
	BranchPredictor(BranchPredictor& other) = default;
	bool predict(uint32_t pc, uint32_t* dst);
	bool update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst);
	SIM_stats getStats();
};

/*
* in this function we update the state of the fsm according to the value of isUpdate
*/
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

/*
/// returning the memory instruction according to the share policy
/// param: pc- contain the memory instruction
/// param: hist- contain the history of the instructions
/// returns the current instuction by the share policy
*/
uint32_t BranchPredictor::getDataBySharePolicy(uint32_t pc, uint32_t hist)
{
	hist = hist & history_size;
	pc = pc >> 2; //start after 2 bytes in the memory instruction

	if (share_state == NO_SHARE) {
		return hist;
	}
	else if (share_state == LSB_SHARE) {
		return (hist ^ pc) & history_size; //(history xor pc) and historySize
	}
	else if (share_state == MID_SHARE) {
		pc = pc >> 14; //start in the 16 bit
		return (hist ^ pc) & history_size; 
	}
	else {
		return hist;
	}
}

/// return the total size according to global history and the global table
/// 
/// param btb_size- the size of the branch predictor
/// param hist_size- the size of the history
/// param tag_size- tag size
/// param is_global_hist
/// param is_global_table
/// returns- total size
unsigned BranchPredictor::getTotalSize(unsigned btb_size, unsigned hist_size, unsigned tag_size,
	bool is_global_hist, bool is_global_table)
{
	unsigned size = btb_size * (1 + tag_size + 30); // 30 is address bit size, 1 for valid bit
	if (is_global_history) {
		size += hist_size;
	}
	else {
		size += (btb_size * hist_size); // local history
	}

	if (is_global_table) {
		size += (2 * pow(2, hist_size));
	}
	else {
		size += (btb_size * 2 * pow(2, hist_size)); // local table
	}
	return size;
}

// return the btb index in the pc
uint32_t BranchPredictor::getBtbIndex(uint32_t pc)
{
	int btb_indexMask = (int)log2(btb_size);
	pc >>= 2;
	return (pc & (uint32_t)(pow(2, btb_indexMask) - 1));
}

//return the tag in the pc
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

	btb_targets = vector<uint32_t>(btbSize, 0);
	btb_tags = vector<uint32_t>(btbSize, INT_MAX);
}

// check if the current instuction is taken or not taken and return the result 
// also, we update the target destination 
// (Taken- true, Not Taken- false)
bool BranchPredictor::predict(uint32_t pc, uint32_t* dst)
{
	bool isBranch = false;
	*dst = pc + 4;
	uint32_t tag = getTag(pc);
	uint32_t index = getBtbIndex(pc);
	uint32_t fsm_table_index = (is_global_table) ? 0 : index;
	uint32_t history_index = (is_global_history) ? 0 : index;

	if (btb_tags[index] != tag) {
		return isBranch; //there is no branch 
	}

	uint32_t history = history_vector[history_index];
	FSM_PREDICTOIN pred = fsm_vector[fsm_table_index][getDataBySharePolicy(pc, history)];
	if (pred == SNT || pred == WNT) {
		isBranch = false; //not taken- there is no branch
	}
	else {
		isBranch = true; // taken- there is a branch
		*dst = btb_targets[index];
	}

	return isBranch;
}

// after we predicting, we update the target and the tag vectors
// check if the destination is not the same with (pc+4 and not taken) or (targetPC and taken) if not we do flush
// return the predicted_result
bool BranchPredictor::update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst)
{
	stats.br_num++;
	uint32_t dst;
	bool predicted_result = predict(pc, &dst);

	if ((taken && dst != targetPc) || (!taken && dst != pc + 4)) {
		stats.flush_num++;
	}

	uint32_t tag = getTag(pc);
	uint32_t index = getBtbIndex(pc);
	uint32_t fsm_table_index = (is_global_table) ? 0 : index;
	uint32_t history_index = (is_global_history) ? 0 : index;
	uint32_t& history = history_vector[history_index];

	if (btb_tags[index] != tag || btb_tags[index] == INT_MAX) {
		btb_targets[index] = targetPc;
		btb_tags[index] = tag;

		if (!is_global_history) {
			history = 0;
		}
		if (!is_global_table) {
			fsm_vector[fsm_table_index] = vector<FSM_PREDICTOIN>(fsm_vector[fsm_table_index].size(), fsm_state);
		}
	}

	btb_targets[index] = targetPc;

	FSM_PREDICTOIN& prediction = fsm_vector[fsm_table_index][getDataBySharePolicy(pc, history)];
	history <<= 1;
	history = (history + taken) & history_size;
	updatePrediction(prediction, taken);
	return predicted_result;
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
	return bp.predict(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	bp.update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
	*curStats = bp.getStats();
}

