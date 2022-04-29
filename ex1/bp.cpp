/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include "branch_predictor.h"

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

