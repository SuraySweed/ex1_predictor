#ifndef BRANCH_PREDICTOR_H_
#define BRANCH_PREDICTOR_H_

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


#endif //BRANCH_PREDICTOR