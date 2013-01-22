/*****************************************************************************
#                      INTEL CONFIDENTIAL
#
# Copyright 2009-2012 Intel Corporation All Rights Reserved.
#
# This file is provided for internal use only. You are free to modify it
# for your own internal purposes, but you cannot release it to the external
# open source community under any circumstances without SSG.DPD's approval.
#
# If you make significant improvements, fix bugs, or have suggestions for
# additional features or improvements, please provide your changes to
# Robert or Gautam for inclusion in this tool.
#
# Please contact Robert Knight (robert.knight@intel.com) or Gautam Upadhyaya
# (gautam.upadhyaya@intel.com) if you have any questions.
#
# ARM port provided by Ekarat Tony Mongkolsmai (ekarat.t.mongkolsmai@intel.com)
*****************************************************************************
*/

/* ******************************************
 * The HT-aware power library.
 * Contains code to read/parse/correlate raw
 * power driver information.
 * ******************************************
 */

#include <stdio.h>
#include <fcntl.h>		/* open */
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <assert.h>

#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <list>
#include <deque>
#include <iterator>
#include <algorithm> // for "std::stable_sort"
#include <sstream>

#ifndef IS_INTEL_INTERNAL
#define IS_INTEL_INTERNAL 0
#endif // IS_INTEL_INTERNAL

#ifndef _ANDROID_
#define _ANDROID_ 0
#endif // _ANDROID_

#include "pw_defines.h"
#include "pw_structs.h" // Essential data structures
#include "pw_arch.h" // Architecture info.
#include "pw_utils.hpp"
#include "wulib_defines.h"
#include "wulib.h"

/* *****************************************
 * Useful defines.
 * *****************************************
 */
/*
 * Get the next token from a string tokenizer.
 */
#define GET_NEXT_TOKEN(tok) ({const char *__tmp = tok.get_next_token(); assert(__tmp != NULL); __tmp;})

#if defined(__linux__)
#define ATOULL(nptr, endptr, base) strtoull(nptr, endptr, base)
#define SNPRINTF(str, size, format, ...) snprintf(str, size, format, __VA_ARGS__)
#define STRNCPY(dstr, sstr, ssize) strncpy(dstr, sstr, ssize)
#elif defined(_WIN32)
#define ATOULL(nptr, endptr, base) (unsigned long long)_strtoui64(nptr, endptr, base)
#define SNPRINTF(str, size, format, ...) _snprintf_s(str, size, size, format, __VA_ARGS__)
#define STRNCPY(dstr, sstr, ssize) strncpy_s(dstr, sizeof(dstr), sstr, ssize)
#endif
#ifdef C0
#undef C0
#endif
#define C0 MPERF

// #define FOR_EACH_CHILD_OF(who, child) for (std::vector<__typeof__(child)>::iterator iter = who->m_children.begin(); iter != who->m_children.end() && (child = *iter); ++iter)
#define FOR_EACH_CHILD_OF(who, child) for (std::vector<Processor *>::iterator iter = who->m_children.begin(); iter != who->m_children.end() && (child = *iter); ++iter)

#define FOR_EACH_MSR(i) for (std::map<int, pw_u64_t>::iterator iter = m_cStateMSRSet.begin(); iter != m_cStateMSRSet.end() && (i = iter->first) >= MPERF; ++iter)

// #define FOR_EACH_PROC(proc, which) for (std::vector<Processor *>::iterator iter = which.begin(); iter != which.end() && (proc = *iter); ++iter)
#define FOR_EACH_PROC(proc, which) for (std::map<int, Processor *>::iterator iter = which.begin(); iter != which.end() && (proc = iter->second); ++iter)

#define GET_MIN_CHILD_STATE() ({ \
    Processor *child = NULL; \
    c_state_t state = C9; \
    FOR_EACH_CHILD_OF(this, child) { \
        if (child->m_state < state) { \
            state = child->m_state; \
        } \
    } \
    state;})

#define GET_MIN_NON_ZERO_CHILD_STATE() ({ \
    Processor *child = NULL; \
    c_state_t state = C9; \
    FOR_EACH_CHILD_OF(this, child) { \
        if (child->m_state > MPERF && child->m_state < state) { \
            state = child->m_state; \
        } \
    } \
    state;})


/*
#define IS_VALID_IDLE(req_state, act_state) ( ((act_state) == MPERF && (req_state) == APERF) || ((act_state) != MPERF && ((act_state) == (req_state) || (PW_IS_ATM() && (act_state) == C5 && (req_state) == C4))) )
*/
/*
#define IS_INVALID_IDLE(req_state, act_state) ( (req_state) == MPERF || ( (act_state) == MPERF && (req_state) != APERF ) )
#define IS_VALID_IDLE(req_state, act_state) ( !IS_INVALID_IDLE(req_state, act_state) )
*/
#define IS_VALID_AUTO_DEMOTE_ENABLED_IDLE(req_state, act_state) ( (act_state) > MPERF || (req_state) > MPERF )
#define IS_VALID_AUTO_DEMOTE_DISABLED_IDLE(req_state, act_state) ( (act_state) > MPERF || (req_state) == APERF )
#define IS_VALID_IDLE(req_state, act_state) ( m_wasAutoDemoteEnabled ? IS_VALID_AUTO_DEMOTE_ENABLED_IDLE(req_state, act_state) : IS_VALID_AUTO_DEMOTE_DISABLED_IDLE(req_state, act_state) )
#define IS_CONFIRMED_IDLE(snapshot) ( (snapshot).cStateSampleIndex >= -1 )

#define GET_REQ_FREQ_FROM_COMBINED(sample) (pw_u32_t)( (sample)->p_sample.unhalted_core_value >> 32 )
#define GET_ACT_FREQ_FROM_COMBINED(sample) (pw_u32_t) ( (sample)->p_sample.unhalted_core_value & 0xffffffff )
#define GET_INT_FROM_FLOAT(f) (int)( (f) + 0.5 )

/* *****************************************
 * Debugging tools.
 * *****************************************
 */
extern bool g_do_debugging;
#define db_fprintf(...) do { \
    if (g_do_debugging) { \
        fprintf(__VA_ARGS__); \
    } \
} while(0)
#define db_assert(e, ...) do { \
    if (g_do_debugging && !(e)) { \
	    fprintf(stderr, __VA_ARGS__);	\
	    assert(false);			\
	}					\
} while(0)
#define db_abort(...) do { \
    if (g_do_debugging) { \
        fprintf(stderr, __VA_ARGS__); \
        assert(false); \
    } \
} while(0)
#define db_copy(...) do { \
    if (g_do_debugging) { \
        std::copy(__VA_ARGS__); \
    } \
} while(0)

/* *****************************************
 * Forward declarations.
 * *****************************************
 */
namespace pwr {
    class Processor;
    class Package;
    class Core;
    class Thread;
    class WuParser;
    struct PStatePredLess;
    struct PStatePredMore;
}

/* *****************************************
 * Some useful typedefs.
 * *****************************************
 */
typedef std::list<PWCollector_sample_t> sample_list_t;
typedef std::vector<std::string> str_vec_t;
typedef std::deque<std::string> str_deq_t;
typedef std::string str_t;
typedef std::vector<FILE *> fp_vec_t;
typedef fp_vec_t::iterator fp_iter_t;
typedef std::vector <int> int_vec_t;

typedef std::pair <int,int> int_pair_t;
typedef std::vector <int_pair_t> pair_vec_t;

typedef std::pair <pw_u64_t, pw_u64_t> pw_u64_pair_t;
typedef std::list <pw_u64_pair_t> p_sample_list_t;
typedef std::map <int, p_sample_list_t> p_sample_map_t;
typedef std::map <int, pw_u64_pair_t> core_tsc_map_t;
typedef std::map <int, std::pair <u64, u64> > aperf_mperf_map_t;

typedef std::vector <pw_u64_t> pw_u64_vec_t;
typedef std::map <int, pw_u64_vec_t> msr_set_map_t;

/* *****************************************
 * Data structure definitions.
 * *****************************************
 */

/*
 * Template specializations to allow
 * us to 'std::list<>::merge(...)' etc.
 */
namespace std {
    template <> struct less<PWCollector_sample_t> {
        bool operator()(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2) {
            return s1.tsc < s2.tsc;
        };
    };
};


namespace pwr {

    struct PStatePredLess {
        const u64& m_collection_start_tsc;
        PStatePredLess(const u64& t) : m_collection_start_tsc(t) {};
        bool operator()(const PWCollector_sample_t& sample) {
            return sample.tsc < m_collection_start_tsc;
        }
    };

    struct PStatePredMore {
        const u64& m_collection_stop_tsc;
        PStatePredMore(const u64& t) : m_collection_stop_tsc(t) {};
        bool operator()(const PWCollector_sample_t& sample) {
            return sample.tsc > m_collection_stop_tsc;
        }
    };

    /*
     * An enumeration of DFA states for a Processor node in the CPU topology
     */
    enum DFA_state {
        DFA_C_ZERO=1, /* All nodes dominated by this node are known to be in C0 */
        DFA_C_HALF, /* Some (but not all) nodes dominated by this node are known to be in idle */
        DFA_C_X /* All nodes dominated by this node are known to be in idle */
    };

    /*
     * Snapshots are sequences in time that capture the DFA state of a given node.
     * This is a snapshot used by Threads.
     */
    struct ThreadSnapshot {
        c_state_t req_state;
        const PWCollector_sample_t *m_sample;
        ThreadSnapshot(const PWCollector_sample_t *sample=NULL) : m_sample(sample), req_state(sample ? (c_state_t)GET_C_STATE_GIVEN_TPS_HINT(sample->c_sample.prev_state) : MPERF) {};

        int get_id() const {
            return (m_sample) ? (int)m_sample->cpuidx : -1;
        };
        c_state_t get_req_state() const {
            return req_state;
        };
        pw_u64_t get_tsc() const {
            return m_sample ? m_sample->tsc : 0x0;
        };
        pw_u64_t get_mperf() const {
            return (m_sample) ? RES_COUNT(m_sample->c_sample, MPERF) : 0x0;
        };
        const PWCollector_sample_t *get_sample() const {
            return m_sample;
        };
    };
    /*
     * Helper function to print out a thread snapshot.
     */
    std::ostream& operator<<(std::ostream& os, const ThreadSnapshot& snapshot)
    {
        os << snapshot.get_tsc() << "\t" << snapshot.get_req_state() << "\n";
        return os;
    };

    /*
     * Helper variable to depict the snapshot of any thread whose status is unknown.
     */
    static const ThreadSnapshot s_emptyThread = ThreadSnapshot(NULL);

    /*
     * Snapshots are sequences in time that capture the DFA state of a given node.
     * This is a snapshot used by cores, modules and packages, and captures the moments in
     * time we believe these kinds of nodes enter idle.
     */
    struct IdleSnapshot {
        const PWCollector_sample_t *tps;
        const PWCollector_sample_t *wu_cause;
        c_state_t req_state, act_state;
        pw_u64_t m_minTSC, m_maxTSC;
        pw_u64_t m_currMinTSC;
        std::map <int, ThreadSnapshot> threadSnapshots;
        int cStateSampleIndex; // index into Processor::m_samples[C_STATE]
        bool m_wasAbort;
        IdleSnapshot(const PWCollector_sample_t *s=NULL, c_state_t r=C9, c_state_t a=C0) : tps(s), req_state(r), act_state(a), wu_cause(NULL), m_minTSC(0), m_maxTSC(0), m_currMinTSC(0), cStateSampleIndex(-2), m_wasAbort(false) {};
    };

    /*
     * Helper function to print out a idle snapshot.
     */
    std::ostream& operator<<(std::ostream& os, const IdleSnapshot& snapshot)
    {
        os << snapshot.tps->tsc << "\t" << snapshot.req_state << "\t" << snapshot.act_state << "\t" << snapshot.m_minTSC << "\t" << snapshot.m_maxTSC << "\t" << snapshot.cStateSampleIndex << "\n";
        for (std::map<int, ThreadSnapshot>::const_iterator citer = snapshot.threadSnapshots.begin(); citer != snapshot.threadSnapshots.end(); ++citer) {
            os << "\t" << citer->first << "->\t" << citer->second;
        }
	return os;
    };

    /*
     * Helper variable: track total number of overcounts.
     */
    static int s_num_pkg_cx_overcounts = 0, s_num_pkg_cx_samples = 0;

    /*
     * A node in the CPU topology.
     */
    class Processor {
        private:
            static int s_count;
        protected:
            /*
             * Architectural flags.
             */
            bool m_wasAutoDemoteEnabled, m_wasAnyThreadSet, m_wasSaltwell;
        protected:
            /*
             * Collection flags.
             */
            bool m_wasCStateCollection;
        protected:
            /*
             * Subtrees and parents.
             */
            std::vector <Processor *> m_children;
            Processor *m_parent;
        protected:
            /*
             * ID fields.
             */
            int m_id, m_uniqueID;
            std::string m_typeString; // One of "Thread", "Core", or "Package"
        protected:
            /*
             * Variables to report this tree's DFA state.
             */
            c_state_t m_currReqState;
            DFA_state m_currDfaState;
            int m_numChildrenInIdle;
        protected:
            /*
             * Variables related to the state of the various C-state MSRs (if any).
             */
            bool m_doesHaveMSRs, m_isFirstTPSAfterIdle, m_isFirstTPS; 
            std::map <int, pw_u64_t> m_cStateMSRSet;
        protected:
            int m_parentIdleSnapshotIdx;
            pw_u64_t m_parentIdleTSC;
            const PWCollector_sample_t *m_parentIdleSample;
        protected:
            pw_u32_t m_currReqFreq, m_currActFreq, m_primeFreq;
        protected:
            pw_u64_t m_parentAbortTSC;
        protected:
            /*
             * Fields used to calculate/store the TSC values of various 'boundary' conditions.
             */
            pw_u64_t m_minTSC, m_maxTSC;
            pw_u64_t m_firstSampleTSC, m_lastSampleTSC;
            pw_u64_t m_beginBoundarySampleTSC, m_endBoundarySampleTSC;
        protected:
            /*
             * Various thread snapshots.
             * Populated only by leaves in the topology tree.
             */
            ThreadSnapshot m_currSnapshot, m_prevSnapshot, m_prevPrevSnapshot;
        protected:
            /*
             * A list of idle snapshots. Populated only by nodes that contain 
             * MSRs (e.g. cores and packages)
             */
            std::vector <IdleSnapshot> m_idleSnapshots;
        protected:
            /*
             * Helper fields to track whether state.
             */
            const PWCollector_sample_t *m_prevTpsSample, *m_FirstNonTpsAfterIdleSample;
        protected:
            /*
             * Helper fields to track whether state.
             */
            const PWCollector_sample_t *m_prevTpfSample;
        protected:
            /*
             * A list of PWCollector_sample instances to output. Filled in by any node that needs to
             * output PWCollector_samples. E.g. Nodes that track MSRs (cores, packages) may have C-state
             * samples to output. Cores will also need to output P-state samples, while packages are used
             * to track other samples (e.g. Wakelock samples, S,D residency samples etc.).
             */
            std::map <sample_type_t, std::list <PWCollector_sample_t> > m_samples;

        protected:
            virtual void dump() const {
                pw_u32_t num_tps = m_samples.find(C_STATE) != m_samples.end() ? (pw_u32_t)m_samples.find(C_STATE)->second.size() : 0;
                pw_u32_t num_tpf = m_samples.find(P_STATE) != m_samples.end() ? (pw_u32_t)m_samples.find(P_STATE)->second.size() : 0;
                if (g_do_debugging) {
                    std::cerr << m_typeString << " " <<  m_id << " (" << m_uniqueID << "): [# tps = " << num_tps << ", # tpf = " << num_tpf << "]\n";
                }
            };

            Processor(const std::string& type, int i) : m_typeString(type), m_id(i), m_uniqueID(s_count++), m_wasAutoDemoteEnabled(pwr::WuData::instance()->getSystemInfo().m_wasAutoDemoteEnabled?true:false), m_wasAnyThreadSet(pwr::WuData::instance()->getSystemInfo().m_wasAnyThreadSet?true:false), m_wasSaltwell(PW_IS_SALTWELL(pwr::WuData::instance()->getSystemInfo().m_cpuModel)?true:false), m_wasCStateCollection(WAS_COLLECTION_SWITCH_SET(pwr::WuData::instance()->getSystemInfo().m_collectionSwitches, PW_POWER_C_STATE)?true:false), m_currReqState(C0), m_currDfaState(DFA_C_ZERO), m_currReqFreq(0x0), m_currActFreq(0x0), m_primeFreq(0x0), m_doesHaveMSRs(false), m_isFirstTPSAfterIdle(false), m_isFirstTPS(true), m_numChildrenInIdle(0), m_parentIdleSnapshotIdx(-1), m_parentIdleTSC(0), m_parentIdleSample(NULL), m_parentAbortTSC(0), m_minTSC(0), m_maxTSC(0), m_firstSampleTSC(0), m_lastSampleTSC(0), m_beginBoundarySampleTSC(0), m_endBoundarySampleTSC(0), m_currSnapshot(), m_prevSnapshot(), m_prevPrevSnapshot(), m_prevTpsSample(NULL), m_FirstNonTpsAfterIdleSample(NULL), m_prevTpfSample(NULL) {};

            virtual ~Processor() {
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    delete child;
                }
                m_children.clear();
            };


        protected:
            bool is_leaf_i() {
                return m_children.empty();
            };

            Processor *get_thread_given_id_i(int cpuid) {
                if (m_children.empty()) { // Leaf ==> Thread
                    return m_id == cpuid ? this : NULL;
                }
                Processor *child = NULL, *retVal = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    retVal = child->get_thread_given_id_i(cpuid);
                    if (retVal) {
                        break;
                    }
                }
                return retVal;
            };

            Processor *get_child_given_id_i(int id) {
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    if (child->m_id == id) {
                        return child;
                    }
                }
                return NULL;
            };

            void dump_tree_freq_state_i(int level)
            {
                for (int i=0; i<level; ++i) {
                    db_fprintf(stderr, "\t");
                }
                db_fprintf(stderr, "%s[%d]: %u, %u\n", m_typeString.c_str(), m_id, m_currReqFreq, m_currActFreq);
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    child->dump_tree_freq_state_i(level+1);
                }
            };

            void update_non_leaf_freq_state_i(const PWCollector_sample_t *sample)
            {
                Processor *child = NULL;
                pw_u32_t __req_freq = 0x0;
                /*
                 * 'Requested' frequency for a non-leaf node is always the
                 * MAX of the frequencies requested by each of its children.
                 */
                FOR_EACH_CHILD_OF(this, child) {
                    __req_freq = std::max(__req_freq, child->m_currReqFreq);
                }
                m_currReqFreq = __req_freq;
                /*
                 * We don't need to infer the actual frequency (UNLESS 'sample' 
                 * depicts a measurement on CLTP, and the frequency of the current
                 * core was involuntarily changed because of the actions of the other core).
                 * Instead, we read that frequency directly.
                 */
                m_currActFreq = GET_ACT_FREQ_FROM_COMBINED(sample);
            };


            bool did_msr_set_change_i(const pw_u64_t msr_set[], c_state_t& act_state, pw_u64_t& cx_res) {
                int i = 0;
                bool retVal = false;
                FOR_EACH_MSR(i) {
                    if (i == APERF) {
                        continue; // APERF couldn't have changed because there's no physical MSR for it!
                    }
                    // db_assert(m_cStateMSRSet[i] <= msr_set[i], "ERROR: C-state MSRs NOT monotonically increasing?!\n"); // C-state MSRs MUST be monotonically increasing
                    if (unlikely(m_cStateMSRSet[i] > msr_set[i])) {
                        db_fprintf(stderr, "ERROR: %s[%d]: C-state MSRs NOT monotonically increasing?!\n", m_typeString.c_str(), m_id);
                        continue;
                    }
                    if (m_cStateMSRSet[i] != msr_set[i]) {
                        cx_res = (msr_set[i] - m_cStateMSRSet[i]);
                        m_cStateMSRSet[i] = msr_set[i];
                        act_state = (c_state_t)i;
                        retVal = true;
                    }
                }
                return retVal;
            };

            void copy_msr_set_i(const pw_u64_t msr_set[]) {
                int i = 0;
                FOR_EACH_MSR(i) {
                    m_cStateMSRSet[i] = msr_set[i];
                }
            };

#define IS_IPI_SAMPLE(sample) ( (sample)->sample_type == IPI_SAMPLE )

            /*
             * INTERNAL API:
             * Function used to adjust a node's DFA state, verify previous node idle enters and exits and generate
             * C-state samples (if required). Note that a node's DFA state depends ONLY on the
             * DFA state of the subtree rooted at this node and on the current sample being serviced.
             * @sample: The sample being serviced.
             */
            void update_non_leaf_dfa_state_i(const PWCollector_sample_t *sample) {
                /*
                 * 'Idle' samples include all samples that can change the 'idle state' of a node. These include
                 * C-state samples and non-C-state samples (e.g. 'IRQ', 'TIMER' etc.). Handle those separately.
                 */
                if (sample->sample_type == C_STATE) {
                    if (m_doesHaveMSRs) {
                        c_state_t act_state = C0;
                        pw_u64_t cx_res = 0;
                        if (unlikely(m_isFirstTPS)) {
                            /*
                             * This is the first C-state sample we've seen. Update our C-state MSR values.
                             */
                            m_isFirstTPS = false;
                            copy_msr_set_i(&RES_COUNT(sample->c_sample, MPERF));
                        } else {
                            if (did_msr_set_change_i(&RES_COUNT(sample->c_sample, MPERF), act_state, cx_res) || m_isFirstTPSAfterIdle) {
                                /*
                                 * This is the first TPS encountered after the last time the core/package entered idle. Use this sample
                                 * to check if the previous idle was 'valid'. Note that an idle is considered INVALID if
                                 * the actual C-state granted wasn't equal to the C-state requested [BUT ONLY if 'auto-demote' is DISABLED!!!]
                                 */
                                if (m_idleSnapshots.empty() == false) {
                                    IdleSnapshot& snapshot = m_idleSnapshots.back();
                                    c_state_t req_state = snapshot.req_state;
                                    if (g_do_debugging) {
                                        std::cerr << "Snapshot TPS: " << *snapshot.tps;
                                        std::cerr << "Req = " << req_state << ", Actual = " << act_state << ", res = " << cx_res << "\n";
                                    }
                                    if (req_state == APERF) {
                                        db_fprintf(stderr, "Found\n");
                                    }
                                    if (is_valid_idle_i(req_state, act_state)) {
                                        /*
                                         * Either a Cx MSR counted or we've confirmed this is a valid C1. In either case,
                                         * set the 'act_state' field of this snapshot.
                                         */
                                        if (act_state == MPERF) {
                                            act_state = APERF;
                                        }
                                        snapshot.act_state = act_state;
                                        if (g_do_debugging) {
                                            std::cerr << m_typeString << "[" << m_id << "]: Confirmed idle at " << sample->tsc << std::endl;
                                        }
                                        /*
                                         * Sanity: MPERF for snapshot->tps == snapshot.threadSnapshots[snapshot->tps->cpuidx].mperf
                                         */
                                        {
                                            pw_u64_t tmp1 = RES_COUNT(snapshot.tps->c_sample, MPERF), tmp2 = snapshot.threadSnapshots[snapshot.tps->cpuidx].get_mperf();
                                            if (tmp1 != tmp2) {
                                                fprintf(stderr, "Warning! Will dump core!\n");
                                            }
                                            assert(RES_COUNT(snapshot.tps->c_sample, MPERF) == snapshot.threadSnapshots[snapshot.tps->cpuidx].get_mperf());
                                        }
                                        create_c_state_sample_from_idle_snapshot_i();
                                        /*
                                         * Ensure the snapshot is considered valid.
                                         */
                                        if (unlikely(IS_CONFIRMED_IDLE(snapshot) == false)) {
                                            assert(m_idleSnapshots.size() == 1);
                                            snapshot.cStateSampleIndex = -1; // snapshot is considered CONFIRMED if index >= -1
                                        }
                                    } else {
                                        /*
                                         * Invalid idle snapshot. Reset it.
                                         */
                                        handle_abort_i(sample);
                                        if (act_state != MPERF && g_do_debugging) {
                                            std::cerr << "BAD snapshot = " << snapshot;
                                            std::cerr << "Current TSC = " << sample->tsc << "\n";
                                            std::cerr << "Invalid idle detected at tsc = " << sample->tsc << std::endl;
                                        }
                                        if (m_wasSaltwell == false) {
                                            if (unlikely(req_state == MPERF)) {
                                                /*
                                                 * Should ONLY happen for the FIRST snapshot, and only because
                                                 * we haven't seen BOTH threads yet. Discard this idle snapshot.
                                                 */
                                                m_idleSnapshots.pop_back();
                                                if (g_do_debugging) {
                                                    std::cerr << "Popped snapshot = " << snapshot;
                                                }
                                            } else {
                                                assert(act_state == MPERF);
                                            }
                                        }
                                    }
                                } else {
                                    db_fprintf(stderr, "No idle snapshots; nothing to do!\n");
                                }
                                m_isFirstTPSAfterIdle = false;
                            }
                        }
                    }
                    /*
                     * OK, previous idle verification done. Now check
                     * if the core/package has entered idle again.
                     */
                    // calculate_dfa_c_states_i();
                    update_dfa_c_states_i(sample);
                    if (g_do_debugging) {
                        std::cerr << m_typeString << "[" << m_id << "] Tsc = " << sample->tsc << ", Curr DFA state = " << m_currDfaState << std::endl;
                    }
                    if (m_doesHaveMSRs) {
                        bool updated_idle = false;
                        /*
                         * Update the previous snapshot if it hasn't yet been confirmed.
                         */
                        if (m_idleSnapshots.empty() == false && IS_CONFIRMED_IDLE(m_idleSnapshots.back()) == false) {
                            IdleSnapshot& snapshot = m_idleSnapshots.back();
                            if (g_do_debugging) {
                                std::cerr << "Before: " << snapshot;
                            }
                            update_idle_snapshot_i(snapshot, sample);
                            if (m_FirstNonTpsAfterIdleSample && (snapshot.wu_cause == NULL || IS_IPI_SAMPLE(snapshot.wu_cause))) {
                                snapshot.wu_cause = m_FirstNonTpsAfterIdleSample;
                            }
                            m_FirstNonTpsAfterIdleSample = NULL;
                            if (g_do_debugging) {
                                std::cerr << "After: " << snapshot;
                            }
                            updated_idle = true;
                        } else {
                            /*
                             * Create a new snapshot now, regardless of whether the core/package actually went idle.
                             * (We can always update the snapshot on the next TPS if it didn't actually enter idle).
                             */
                            IdleSnapshot idleSnapshot(sample);
                            create_idle_snapshot_i(idleSnapshot, sample->cpuidx);
                            idleSnapshot.wu_cause = m_FirstNonTpsAfterIdleSample;
                            m_FirstNonTpsAfterIdleSample = NULL;
                            m_idleSnapshots.push_back(idleSnapshot);
                            if (g_do_debugging) {
                                std::cerr << "Created idle snapshot: " << idleSnapshot;
                            }
                        }
                        /*
                         * Also check if we need to set the 'first idle' flag.
                         */
                        if (m_currDfaState == DFA_C_X) {
                            m_isFirstTPSAfterIdle = true;
                            if (g_do_debugging) {
                                std::cerr << m_typeString << "[" << m_id << "]: Set first after idle at tsc = " << sample->tsc << std::endl;
                            }
                        }
                    }
                    m_prevTpsSample = sample;
                } else { // non-TPS
                    // calculate_dfa_c_states_i();
                    update_dfa_c_states_i(sample);
                    if (likely(m_isFirstTPS == false)) { // We've already seen at least one TPS sample for this core/package
                        if (m_FirstNonTpsAfterIdleSample == NULL || IS_IPI_SAMPLE(m_FirstNonTpsAfterIdleSample)) {
                            m_FirstNonTpsAfterIdleSample = sample;
                        }
                    }
                }
            };

            void notify_children_of_idle_i(const IdleSnapshot *snapshot) {
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    child->m_parentIdleTSC = snapshot->tps->tsc;
                    child->m_parentIdleSample = snapshot->tps;
                }
            };
            void notify_children_of_idle_i(const PWCollector_sample_t *sample) {
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    child->m_parentIdleTSC = sample->tsc;
                    child->m_parentIdleSample = sample;
                }
            };

            void notify_child_of_abort_i(Processor *child, const pw_u64_t& tsc) {
                child->m_parentAbortTSC = tsc;
            };

            virtual int handle_idle_sample_i(const PWCollector_sample_t *sample) {
                return PW_SUCCESS;
            };

            virtual int handle_freq_sample_i(const PWCollector_sample_t *sample) {
                m_currReqFreq = GET_REQ_FREQ_FROM_COMBINED(sample);
                m_currActFreq = GET_REQ_FREQ_FROM_COMBINED(sample);
                return PW_SUCCESS;
            };

            virtual int handle_cpuhotplug_sample_i(const PWCollector_sample_t *sample) {
                return PW_SUCCESS;
            };

            virtual void push_samples_to_cores_i() {
                // NOP
            };

            virtual void add_topology_mask_to_cpuid_i(pw_u32_t& cpuid) {
                // NOP
            };

            virtual bool is_valid_idle_i(c_state_t req_state, c_state_t act_state) {
                if (m_wasAutoDemoteEnabled) {
                    return IS_VALID_AUTO_DEMOTE_ENABLED_IDLE(req_state, act_state);
                }
                return IS_VALID_AUTO_DEMOTE_DISABLED_IDLE(req_state, act_state);
            };

            virtual void handle_abort_i(const PWCollector_sample_t *sample) {
                // NOP
            };

            virtual void take_thread_snapshot_i(const PWCollector_sample_t *sample) {
                // NOP
            };

            virtual void create_idle_snapshot_i(IdleSnapshot& idleSnapshot, int curr_tps_cpuid) {
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    child->create_idle_snapshot_i(idleSnapshot, curr_tps_cpuid);
                }
            };


            void update_idle_snapshot_i(IdleSnapshot& snapshot, const PWCollector_sample_t *sample) {
                /*
                 * Need to update:
                 * 1. TPS sample
                 * 2. thread snapshot of current thread i.e. of sample->cpuidx
                 * 3. Requested state.
                 * 4. Min/Max TSCs
                 */
                snapshot.tps = sample;
                Processor *thread = get_thread_given_id_i(sample->cpuidx);
                assert(thread);
                const ThreadSnapshot& thread_snapshot = thread->m_currSnapshot;
                pw_u64_t thread_tsc = thread_snapshot.get_tsc();
                snapshot.threadSnapshots[thread->m_id] = thread_snapshot;
                if (m_wasSaltwell /*PW_IS_ATM(pwr::WuData::instance()->getSystemInfo().m_arch)*/) {
                    snapshot.req_state = C9;
                    for (std::map <int, ThreadSnapshot>::const_iterator citer = snapshot.threadSnapshots.begin(); citer != snapshot.threadSnapshots.end(); ++citer) {
                        snapshot.req_state = std::min(snapshot.req_state, citer->second.get_req_state());
                    }
                } else { // not ATM
                    snapshot.req_state = m_currReqState;
                }
                if (m_currDfaState != DFA_C_X) {
                    snapshot.m_currMinTSC = thread_tsc;
                } else {
                    if (snapshot.m_currMinTSC == 0 || thread_tsc < snapshot.m_currMinTSC) {
                        snapshot.m_currMinTSC = thread_tsc;
                    }
                }
                if (snapshot.m_minTSC == 0 || thread_tsc < snapshot.m_minTSC) {
                    snapshot.m_minTSC = thread_tsc;
                }
                snapshot.m_maxTSC = std::max(snapshot.m_maxTSC, thread_tsc);
            };
            virtual void update_dfa_c_states_i(const PWCollector_sample_t *sample) {
                calculate_dfa_c_states_i();
            };
            /*
             * INTERNAL API: calculate this node's DFA state.
             */
            virtual void calculate_dfa_c_states_i() {
                /*
                 * Non-leaf node: next DFA state governed by state of children.
                 */
                int num_children_in_idle = 0;
                Processor *child = NULL;
                c_state_t minChildState = MAX_MSR_ADDRESSES;
                FOR_EACH_CHILD_OF(this, child) {
                    db_fprintf(stderr, "[%d]: %d\n", child->m_id, child->m_currDfaState);
                    if (child->m_currDfaState == DFA_C_X) {
                        ++num_children_in_idle;
                        minChildState = std::min(minChildState, child->m_currReqState);
                    }
                }
                db_fprintf(stderr, "# children in idle = %d\n", num_children_in_idle);
                if (num_children_in_idle == m_children.size()) {
                    /*
                     * All our children are in idle.
                     */
                    m_currDfaState = DFA_C_X;
                    m_currReqState = minChildState;
                } else {
                    /*
                     * Check if at least one of our children is in idle. If so, set our state
                     * to 'DFA_C_HALF'. If not, we're in 'DFA_C_ZERO'
                     */
                    m_currDfaState = (num_children_in_idle == 0) ? DFA_C_ZERO : DFA_C_HALF;
                    // m_currReqState = C0;
                }
            };

            /*
             * INTERNAL API: reset this node's DFA state.
             */
            void reset_dfa_states_i() {
                Processor *child = NULL;
                // m_currReqState = C0;
                m_currDfaState = DFA_C_ZERO;
                FOR_EACH_CHILD_OF(this, child) {
                    child->reset_dfa_states_i();
                }
            };

            /*
             * INTERNAL API: Create a new C-state sample from a series of idle snapshots.
             * @index: index into a vector of idle snapshots. The C-state sample is created between
             * vector[index] and vector[index-1]
             */
            void create_c_state_sample_from_idle_snapshot_i() {
                int size = (int)m_idleSnapshots.size();
                int index = size - 1;
                assert(size);
                if (size < 2 || index == 0 || index >= size) {
                    /*
                     * We don't have enough idle snapshots to create a new C-state sample. However, we
                     * must still tell our children that we entered idle here.
                     */
                    notify_children_of_idle_i(m_idleSnapshots.back().tps);
                    return;
                }
                const IdleSnapshot& prev = m_idleSnapshots[index-1];
                IdleSnapshot& curr = m_idleSnapshots[index];
                int cpuidx = curr.tps->cpuidx;
                c_state_t req_state = prev.req_state, act_state = prev.act_state;
                pw_u64_t delta_tsc = curr.tps->tsc - prev.tps->tsc, c0_res = 0, cx_res = 0;
                if (m_typeString == "Package") {
                    if (curr.m_currMinTSC <= prev.m_maxTSC) {
                        fprintf(stderr, "CURR MIN IS LESS THAN PREV MAX! curr min = %llu, prev max = %llu\n", curr.m_currMinTSC, prev.m_maxTSC);
                        std::cerr << "Curr snapshot = " << curr;
                        std::cerr << "Prev snapshot = " << prev;
                        assert(false);
                    } else if (curr.m_currMinTSC <= prev.tps->tsc) {
                        fprintf(stderr, "CURR MIN IS LESS THAN PREV TPS!\n");
                        assert(false);
                    }
                }
                if (unlikely(req_state == MPERF)) {
                    /*
                     * Usually happens because we've seen a TPS from only one thread so far.
                     * Assume the requested state == actual state in this case (usually a safe
                     * assumption).
                     */
                    req_state = act_state;
                }
                if (unlikely(prev.m_wasAbort)) {
                    /*
                     * For an 'abort', we set the 'act_state' == MPERF, the 'Cx' residency to zero
                     * and the  'C0' residency to delta-TSC.
                     */
                    assert(act_state == APERF);
                    c0_res = delta_tsc; cx_res = 0x0;
                } else {
                    switch (act_state) {
                        case APERF:
                            act_state = APERF;
                            c0_res = curr.threadSnapshots.find(cpuidx)->second.get_mperf() - prev.threadSnapshots.find(cpuidx)->second.get_mperf();
#if 1
                            if (c0_res >= delta_tsc) {
                                // c0_res = delta_tsc - 1;
                                c0_res = delta_tsc;
                                db_fprintf(stderr, "C1 warning: tsc = %llu\n", curr.tps->tsc);
                                // ++s_num_pkg_cx_overcounts;
                            }
#else
                            {
                                pw_u64_t __tsc_delta = (curr.m_currMinTSC - prev.tps->tsc);
                                if (c0_res >= __tsc_delta) {
                                    c0_res = __tsc_delta - 1;
                                    assert(c0_res < delta_tsc);
                                    db_fprintf(stderr, "C1 warning: tsc = %llu\n", curr.tps->tsc);
                                }
                            }
#endif
                            cx_res = delta_tsc - c0_res;
                            break;
                        default:
                            cx_res = RES_COUNT(curr.tps->c_sample, act_state) - RES_COUNT(prev.tps->c_sample, act_state);
                            cx_res *= C_STATE_RES_COUNT_MULTIPLIER();
#if 0
                            if (cx_res >= delta_tsc) {
                                db_fprintf(stderr, "CX warning: state = %u, cx res = %llu delta tsc = %llu curr tsc = %llu prev tsc = %llu\n", act_state, cx_res, delta_tsc, curr.tps->tsc, prev.tps->tsc);
                                db_fprintf(stderr, "CURR MIN = %llu\n", curr.m_minTSC); 
                                cx_res = delta_tsc - 1;
                                if (m_wasSaltwell) {
                                    ++s_num_pkg_cx_overcounts;
                                }
                            }
#else
                            {
                                pw_u64_t __tsc_delta = (curr.m_minTSC - prev.tps->tsc);
#if 0
                                if (curr.wu_cause && curr.wu_cause->tsc < curr.m_minTSC) {
                                    if (m_typeString == "Package") {
                                        db_fprintf(stderr, "At TSC = %llu, USING wu_cause tsc = %llu instead of min TPS tsc = %llu\n", curr.tps->tsc, curr.wu_cause->tsc, curr.m_minTSC);
                                    }
                                    assert(curr.wu_cause->tsc > prev.tps->tsc);
                                    __tsc_delta = (curr.wu_cause->tsc - prev.tps->tsc);
                                } else {
                                    if (m_typeString == "Package") {
                                        db_fprintf(stderr, "NOT using wu_cause tsc at TSC = %llu!\n", curr.tps->tsc);
                                    }
                                }
#endif
                                if (cx_res >= __tsc_delta) {
                                    cx_res = __tsc_delta - 1;
                                    db_assert(cx_res < delta_tsc, "ERROR during processing in FILE %s LINE %d: possible inconsistent input data. RECOMMEND YOU REBOOT YOUR TARGET DEVICE!\n", __FILE__, __LINE__);
                                    if (cx_res >= delta_tsc) {
                                        fprintf(stderr, "ERROR during processing in FILE %s LINE %d: possible inconsistent input data. RECOMMEND YOU REBOOT YOUR TARGET DEVICE!\n", __FILE__, __LINE__);
                                        exit(-PW_ERROR);
                                    }
                                    // assert(cx_res < delta_tsc);
                                    if (m_wasSaltwell) {
                                        ++s_num_pkg_cx_overcounts;
                                    }
                                }
                            }
#endif
                            c0_res = delta_tsc - cx_res;
                            if (m_wasSaltwell) {
                                ++s_num_pkg_cx_samples;
                            }
                            break;
                    }
                }
                const PWCollector_sample_t *wu_cause = curr.wu_cause;
                PWCollector_sample_t tps_sample;
                create_c_state_sample_i(tps_sample, curr.tps->tsc, req_state, act_state, c0_res, cx_res, wu_cause);
                if (unlikely(cx_res == 0x0 && tps_sample.c_sample.break_type != PW_BREAK_TYPE_B)) {
                    tps_sample.c_sample.break_type = PW_BREAK_TYPE_A;
                }
                if (unlikely(prev.m_wasAbort)) {
                    assert(tps_sample.c_sample.break_type == PW_BREAK_TYPE_A); // sanity!
                }
                m_samples[C_STATE].push_back(tps_sample);
                curr.cStateSampleIndex = -1; // snapshot is considered CONFIRMED if index >= -1
                /*
                 * Our algorithm only really requires the two most recent snapshots to be present at any given point
                 * in time. For safety, we retain the last 5.
                 */
                while ( (size = (int)m_idleSnapshots.size()) > 5) {
                    m_idleSnapshots.erase(m_idleSnapshots.begin());
                }
                /*
                */
                /*
                 * We'll need to notify our children that we entered idle.
                 */
                m_parentIdleSnapshotIdx = (int)m_idleSnapshots.size() - 1;
            };

            /*
             * INTERNAL API: Create a new C-state sample.
             * @tps_sample: reference to the newly created sample.
             * @tsc, @req_state, @act_state, @c0_res, @cx_res: various parameters for @tps_sample
             * @wu_cause: wakeup cause to assign @tps_sample.
             */
            void create_c_state_sample_i(PWCollector_sample_t& tps_sample, const pw_u64_t& tsc, c_state_t req_state, c_state_t act_state, const pw_u64_t& c0_res, const pw_u64_t& cx_res, const PWCollector_sample_t *wu_cause) {
                memset(&tps_sample, 0, sizeof(tps_sample));

                tps_sample.cpuidx = m_id;
                tps_sample.sample_type = C_STATE;
                tps_sample.tsc = tsc;

                tps_sample.c_sample.prev_state = (pw_u16_t)(((pw_u8_t)req_state) << 8 | (pw_u8_t)act_state);
                db_fprintf(stderr, "req = %u, act = %u, prev_state = %u\n", req_state, act_state, tps_sample.c_sample.prev_state);

                RES_COUNT(tps_sample.c_sample, MPERF) = c0_res; RES_COUNT(tps_sample.c_sample, act_state) = cx_res;
                tps_sample.c_sample.break_type = PW_BREAK_TYPE_U;

                if (wu_cause != NULL) {
                    set_wakeup_cause_i(tps_sample, wu_cause);
                } else {
                    /*
                     * We no longer need the 'EPOCH' field.
                     * Use it to store the CPUID of the wakeup event.
                     */
                    /*
                    tps_sample.c_sample.tps_epoch = tps_sample.cpuidx;
                    */
                    int __wu_cpuid = tps_sample.cpuidx;
                    if (likely(m_children.size())) {
                        __wu_cpuid = m_children.front()->get_id();
                    }
                    tps_sample.c_sample.tps_epoch = __wu_cpuid;
                }

                if (g_do_debugging) {
                    std::cerr << tps_sample;
                }

                /*
                 * Add a bitmask to the most significant bits of the 'cpuidx' field to indicate
                 * whether this is a core, module, or package c-state sample.
                 */
                add_topology_mask_to_cpuid_i(tps_sample.cpuidx);
            };

            void set_wakeup_cause_i(PWCollector_sample_t& tps_sample, const PWCollector_sample_t *wu_sample) {
                int wakeup_cpuidx = wu_sample->cpuidx;
                pw_u8_t break_type = (pw_u8_t)wu_sample->sample_type;
                /*
                 * Need to convert 'break_type' to its 'PW_BREAK_TYPE_xxx' equivalent.
                 * e.g. "TIMER_SAMPLE" <--> PW_BREAK_TYPE_T etc.
                 */
                switch (break_type) {
                    case FREE_SAMPLE:
                        /* Only happens if we're inserting a 'Ghost' sample. */
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_B;
                        break;
                    case TIMER_SAMPLE:
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_T;
                        tps_sample.c_sample.pid = (pid_t)wu_sample->e_sample.data[0];
                        tps_sample.c_sample.tid = (pid_t)wu_sample->e_sample.data[1];
                        tps_sample.c_sample.c_data = wu_sample->e_sample.data[2];
                        break;
                    case IRQ_SAMPLE:
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_I;
                        tps_sample.c_sample.pid = tps_sample.c_sample.tid = 0;
                        tps_sample.c_sample.c_data = wu_sample->e_sample.data[0];
                        break;
                    case WORKQUEUE_SAMPLE:
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_W;
                        break;
                    case SCHED_SAMPLE:
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_S;
                        /*
                         * For SCHED_WAKEUP events, we need the CPUID of the
                         * (Logical) CPU that was TARGETED, and NOT the CPUID
                         * of the (Logical) CPU that sent the SCHED_WAKEUP!
                         */
                        wakeup_cpuidx = (int)wu_sample->e_sample.data[1]; // TARGET cpu!
                        tps_sample.c_sample.c_data = wu_sample->e_sample.data[0]; // SOURCE cpu == wu_sample->cpuidx!
                        break;
                    case IPI_SAMPLE:
                        /*
                         * Update: changing 'IPI' to 'UNKNOWN', per Bob's request.
                         */
                        // tps_sample.c_sample.break_type = PW_BREAK_TYPE_IPI;
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_U;
                        break;
                    default:
                        db_fprintf(stderr, "Warning: Break type = %d\n", break_type);
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_U;
                        break;
                }
                /*
                 * We no longer need the 'EPOCH' field.
                 * Use it to store the CPUID of the wakeup event.
                 */
                tps_sample.c_sample.tps_epoch = wakeup_cpuidx;
                if (g_do_debugging) {
                    std::cerr << "Wu sample: " << *wu_sample;
                    std::cerr << "Set wakeup cause: " << tps_sample;
                }
            };

            void create_p_state_sample_i(PWCollector_sample_t& tpf_sample, const pw_u64_t& tsc, pw_u32_t frequency, pw_u32_t boundary_val);
            void prune_tpf_samples_i(const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc, pw_u32_t& collection_start_freq, pw_u32_t& collection_stop_freq);

        public:
            /*
             * 'PROC_MAP' and/or 'DEV_MAP' samples *MAY* have been sent by the wuwatch executable itself
             * (at the beginning of the collection). Additionally, 'K_CALL_STACK' samples MAY have 
             * 1 <= {TSC} <= (# logical processors), so don't use 'K_CALL_STACK' samples for TSC values
             * either.
             */
#define SHOULD_CHECK_SAMPLE_FOR_FIRST_TSC(sample) !( (sample)->sample_type == PROC_MAP || (sample)->sample_type == DEV_MAP || (sample)->sample_type == K_CALL_STACK || (sample)->sample_type == PKG_MAP || (sample)->sample_type == W_STATE )
            /*
             * PUBLIC API:
             * Main sample handler. Take appropriate actions based on the type of sample we're seeing.
             * All samples are GUARANTEED to traverse the entire topology tree, from a leaf to
             * the root!
             * @sample: the sample to 'handle'.
             *
             * @returns: 0 on success, -1 on error
             */
            int handle_sample(const PWCollector_sample_t *sample) {
                /*
                 * Collection START TSC is defined as the first sample
                 * sent by the driver.
                 */
                if (unlikely(m_firstSampleTSC == 0)) {
                    if (SHOULD_CHECK_SAMPLE_FOR_FIRST_TSC(sample)) {
                        m_firstSampleTSC = sample->tsc;
                    }
                }
                /*
                 * We also keep track of the last sample TSC. This is in case we requested
                 * a C-state collection, but the core/package never entered idle.
                 */
                /*
                if (m_wasCStateCollection) {
                    if (sample->sample_type == C_STATE) {
                        m_lastSampleTSC = sample->tsc;
                    }
                } else {
                    // Not a C-state collection; no need to check for sample type
                    m_lastSampleTSC = sample->tsc;
                }
                */
                if (SHOULD_CHECK_SAMPLE_FOR_FIRST_TSC(sample)) {
                    m_lastSampleTSC = sample->tsc;
                }
                sample_type_t sample_type = (sample_type_t)sample->sample_type;
                switch (sample_type) {
                    case C_STATE:
                    case TIMER_SAMPLE:
                    case IRQ_SAMPLE:
                    case WORKQUEUE_SAMPLE:
                    case SCHED_SAMPLE:
                    case IPI_SAMPLE:
                        /*
                         * These are 'idle' samples, because they affect the 'idle state' of this node
                         * (e.g. a "C_STATE" sample indicates the node may be entering idle, while an "IRQ"
                         * sample indicates the node is NOT in idle).
                         */
                        if (handle_idle_sample_i(sample)) {
                            return -PW_ERROR;
                        }
                        break;
                    case P_STATE:
                        if (handle_freq_sample_i(sample)) {
                            return -PW_ERROR;
                        }
                        break;
                    case CPUHOTPLUG_SAMPLE:
                        if (handle_cpuhotplug_sample_i(sample)) {
                            return -PW_ERROR;
                        }
                        break;
                    default:
                        /*
                         * Default action is to simply store the samples if we're 
                         * the root node, and ignore it otherwise.
                         */
                        if (m_parent == NULL) {
                            m_samples[sample_type].push_back(*sample);
                        }
                        break;
                }
                return m_parent ? m_parent->handle_sample(sample) : PW_SUCCESS;
            };

            virtual void get_rewound_thread_snapshot(ThreadSnapshot& snapshot, const pw_u64_t tsc) {
                // NOP
            };

            virtual const PWCollector_sample_t *get_rewound_sample(const pw_u64_t tsc) {
                return NULL; // NOP
            };

            int get_id() const {
                return m_id;
            };

            DFA_state get_current_dfa_state() const {
                return m_currDfaState;
            };

            pw_u32_t get_curr_req_freq() const {
                return m_currReqFreq;
            };

            pw_u32_t get_curr_act_freq() const {
                return m_currActFreq;
            };


            pw_u32_t get_prime_freq() const {
                return m_primeFreq;
            };

            int get_first_leaf_of() {
                if (m_children.empty()) {
                    return m_id;
                }
                Processor *child = NULL;
                FOR_EACH_CHILD_OF(this, child) {
                    int __tmp = child->get_first_leaf_of();
                    if (__tmp >= 0) {
                        return __tmp;
                    }
                }
                return -1;
            };


            void set_min_max_collection_tscs(pw_u64_t& min_first_sample_tsc, pw_u64_t& max_last_sample_tsc, pw_u64_t& min_c_state_tsc, pw_u64_t& max_c_state_tsc);
            void finalize_other_samples(std::list <PWCollector_sample_t>& samples, sample_type_t type) const;
            void finalize_tps_samples(std::list <PWCollector_sample_t>& samples, const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc);
            void finalize_tpf_samples(std::list <PWCollector_sample_t>& samples, const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc);
            void append_samples(const std::list <PWCollector_sample_t>& samples, const sample_type_t& type);
            void merge_c_state_samples(const std::list <PWCollector_sample_t>& samples);
            void merge_p_state_samples(const std::list <PWCollector_sample_t>& samples);
            void push_samples_to_cores();

            void add_child(Processor *child);
            void set_msrs(const std::vector<int>& supported_msrs);
            void dfs_traverse(int level) const;

            std::string name() const;

    };

    int Processor::s_count = 0;


    /*
     * INTERNAL API:
     * Create a new P-state sample.
     */
    void Processor::create_p_state_sample_i(PWCollector_sample_t& tpf_sample, const pw_u64_t& tsc, pw_u32_t frequency, pw_u32_t boundary_val) {
        memset(&tpf_sample, 0, sizeof(tpf_sample));

        tpf_sample.cpuidx = m_id;
        tpf_sample.sample_type = P_STATE;
        tpf_sample.tsc = tsc;

        tpf_sample.p_sample.prev_req_frequency = frequency;
        tpf_sample.p_sample.frequency = frequency;
        tpf_sample.p_sample.is_boundary_sample = boundary_val;
    };

    /*
     * INTERNAL API:
     * Remove duplicates and out-of-bounds P-state samples.
     */
    void Processor::prune_tpf_samples_i(const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc, pw_u32_t& collection_start_freq, pw_u32_t& collection_stop_freq) {
        db_fprintf(stderr, "%s[%d]: Pruning, start tsc = %llu, stop tsc = %llu\n", m_typeString.c_str(), m_id, collection_start_tsc, collection_stop_tsc);
        if (m_samples[P_STATE].empty()) {
            return;
        }
        pw_u32_t beg_freq = m_samples[P_STATE].front().p_sample.frequency, end_freq = m_samples[P_STATE].back().p_sample.frequency;
        std::list<PWCollector_sample_t>::iterator curr = m_samples[P_STATE].begin(), next = curr;

        /*
         * Sanity!
         */
        if (false) {
            pw_u32_t last_freq = beg_freq;
            pw_u64_t last_tsc = curr->tsc;
            for (++curr; curr != m_samples[P_STATE].end(); last_freq = curr->p_sample.frequency, last_tsc = curr->tsc, ++curr) {
                if (curr->p_sample.frequency == last_freq) {
                    fprintf(stderr, "Warning: tscs %llu and %llu have the same frequency = %u\n", last_tsc, curr->tsc, last_freq);
                }
            }
            assert(false);
        }

        collection_start_freq = 0; collection_stop_freq = 0;

        for (++next; next != m_samples[P_STATE].end(); curr = next, ++next) {
            if (collection_start_freq == 0 && curr->tsc >= collection_start_tsc) {
                collection_start_freq = curr->p_sample.frequency;
            }
            if (collection_stop_freq == 0 && curr->tsc >= collection_stop_tsc) {
                collection_stop_freq = curr->p_sample.frequency;
            }
            if (curr->p_sample.frequency == next->p_sample.frequency) {
                /* Remove 'curr' */
                if (g_do_debugging) {
                    std::cerr << "Erasing: " << *curr;
                }
                m_samples[P_STATE].erase(curr);
            } else {
                /* Retain 'curr' */
            }
        }
        if (collection_start_freq == 0) {
            /*
             * Can only happen if the collection_start_tsc was greater than all
             * the P-state TSCs. In this case we ASSUME the frequency doesn't
             * change between the last P-state sample and the collection start.
             */
            collection_start_freq = end_freq;
        }
        if (collection_stop_freq == 0) {
            /*
             * Can only happen if the collection_stop_tsc was greater than all
             * the P-state TSCs. In this case we ASSUME the frequency doesn't
             * change between the last P-state sample and the collection stop.
             */
            collection_stop_freq = end_freq;
        }
        /*
         * OK, we've calculated the P-state transitions. Now eliminate all those
         * that don't fall between our previously calculated collection limits.
         */
        m_samples[P_STATE].remove_if(PStatePredLess(collection_start_tsc));
        m_samples[P_STATE].remove_if(PStatePredMore(collection_stop_tsc));
    };

    /*
     * Calculate collection start and stop time aka the collection 'boundaries'.
     * @min_first_sample_tsc: (calculated) TSC of first sample sent by driver.
     * @min_c_state_tsc: (calculated) TSC of first 'mwait' on this thread/core/package.
     * @max_c_state_tsc: (calculated) TSC of last 'mwait' on this thread/core/package.
     */
    void Processor::set_min_max_collection_tscs(pw_u64_t& min_first_sample_tsc, pw_u64_t& max_last_sample_tsc, pw_u64_t& min_c_state_tsc, pw_u64_t& max_c_state_tsc) {
        if (m_firstSampleTSC > 0) {
            if (min_first_sample_tsc == 0 || m_firstSampleTSC < min_first_sample_tsc) {
                /*
                 * AXE cannot handle zero residencies. This means the ghost/boundary sample(s) MUST have non-zero C0, Cx residencies. To
                 * ensure this, subtract 1 from the min TSC. Note that we only need to do this for the begin boundary sample.
                 */
                min_first_sample_tsc = (m_firstSampleTSC - 1);
            }
        }
        max_last_sample_tsc = std::max(max_last_sample_tsc, m_lastSampleTSC);
        if (m_samples[C_STATE].empty() == false) {
            const PWCollector_sample_t& first_tps = m_samples[C_STATE].front(), &last_tps = m_samples[C_STATE].back();
            c_state_t __act_state = (c_state_t)GET_ACT_FROM_PREV(first_tps.c_sample.prev_state);
            pw_u64_t __c0_res = RES_COUNT(first_tps.c_sample, MPERF);
            pw_u64_t __cx_res = RES_COUNT(first_tps.c_sample, __act_state);
            if (unlikely(PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel) && __act_state == APERF)) {
                RESET_C1I_FLAG(__cx_res);
            }
            /*
             * First 'mwait' TSC == (TSC of first c-state sample) - (C0+Cx res of first c-state sample)
             */
            m_minTSC = min_c_state_tsc = first_tps.tsc - __c0_res - __cx_res;
            /*
             * Last 'mwait' TSC == (TSC of last c-state sample)
             */
            m_maxTSC = max_c_state_tsc = last_tps.tsc;
        } else {
            /*
             * No TPS samples, but are there any TPF samples?
             */
            if (m_prevTpfSample) { // Need to do this to ensure we get the last TPF sample
                m_samples[P_STATE].push_back(*m_prevTpfSample);
                std::cerr << *m_prevTpfSample;
                m_prevTpfSample = NULL;
                /*
                 * Ensure all TPF samples have the core ID in the 'cpuidx' field.
                 */
                m_samples[P_STATE].back().cpuidx = m_id;
            }
            if (m_samples[P_STATE].empty() == false) {
                m_minTSC = min_c_state_tsc = m_samples[P_STATE].front().tsc;
                m_maxTSC = max_c_state_tsc = m_samples[P_STATE].back().tsc;
            }
        }
        for (unsigned i=0; i<m_children.size(); ++i) {
            pw_u64_t child_min_tsc = 0, child_max_tsc = 0;
            m_children[i]->set_min_max_collection_tscs(min_first_sample_tsc, max_last_sample_tsc, child_min_tsc, child_max_tsc);
            if (min_c_state_tsc == 0 || (child_min_tsc > 0 && child_min_tsc < min_c_state_tsc)) {
                min_c_state_tsc = child_min_tsc;
            }
            max_c_state_tsc = std::max(max_c_state_tsc, child_max_tsc);
        }
        if (min_first_sample_tsc == 0) {
            min_first_sample_tsc = min_c_state_tsc;
        }
    };

    void Processor::finalize_other_samples(std::list <PWCollector_sample_t>& samples, sample_type_t type) const {
        if (m_samples.find(type) != m_samples.end()) {
            const sample_list_t& list = m_samples.find(type)->second;
            samples.insert(samples.end(), list.begin(), list.end());
        }
        for (unsigned i=0; i<m_children.size(); ++i) {
            m_children[i]->finalize_other_samples(samples, type);
        }
    };
    /*
     * 'Finalize' tps samples: add 'boundary' C-state samples (if required).
     * @samples: the list of C-state samples to output.
     * @collection_start_tsc, @collection_stop_tsc: calculated collection boundaries
     */
    void Processor::finalize_tps_samples(std::list <PWCollector_sample_t>& samples, const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc) {
        pw_u32_t collectionSwitches = pwr::WuData::instance()->getSystemInfo().m_collectionSwitches;
        if (WAS_COLLECTION_SWITCH_SET(collectionSwitches, PW_POWER_C_STATE)) {
            if (m_minTSC && m_maxTSC) {
                if (m_samples[C_STATE].empty() == false) {
                    /*
                     * Need a begin and (probably) an end BOUNDARY sample.
                     */
                    if (likely(collection_start_tsc < m_minTSC)) {
                        /*
                         * Add a "ghost" wakeup sample at the beginning to adjust for
                         * C0+C1 time. This wakeup sample will have the following
                         * characteristics:
                         * 1. TSC == m_minTSC;
                         * 2. C0 res == (m_minTSC - collection_start_tsc) - 1 // we subtract one to account for the 1-cycle 'Cx' residency; see below
                         * 3. Cx res == 1 // We can't have a ZERO residency because that breaks AXE!!!
                         * 4. 'Which Cx' == {Lowest sleep state allowed by the user, defaults to APERF/C1}
                         * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                         * 6. Wakeup Cause == 'BOGUS'
                         * ---------------------------------------------------------------
                         * UPDATE: do this ONLY if the user asked for C-state samples!!!
                         * ---------------------------------------------------------------
                         */
                        pw_u64_t cx_res = 1;
                        pw_u64_t c0_res = (m_minTSC - collection_start_tsc) - cx_res;
                        PWCollector_sample_t tps_sample = {m_id};
#if 0
                        PWCollector_sample_t tmp_wakeup_sample = {m_id};
                        create_c_state_sample_i(tps_sample, m_minTSC, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, &tmp_wakeup_sample);
#endif
                        create_c_state_sample_i(tps_sample, m_minTSC, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, NULL);
                        m_samples[C_STATE].push_front(tps_sample);
                    }
                    if (likely(collection_stop_tsc > m_maxTSC)) {
                        /*
                         * Add a "ghost" wakeup sample at the end to adjust for
                         * C0+C1 time. This wakeup sample will have the following
                         * characteristics:
                         * 1. TSC == collection_stop_tsc;
                         * 2. C0 res == (collection_stop_tsc - m_maxTSC) - 1 // we subtract one to account for the 1-cycle 'Cx' residency; see below
                         * 3. Cx res == 1 // We can't have a ZERO residency because that breaks AXE!!!
                         * 4. 'Which Cx' == {Lowest sleep state allowed by the user, defaults to APERF/C1}
                         * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                         * 6. Wakeup Cause == 'BOGUS'
                         * ---------------------------------------------------------------
                         * UPDATE: do this ONLY if the user asked for C-state samples!!!
                         * ---------------------------------------------------------------
                         */
                        pw_u64_t cx_res = 1;
                        pw_u64_t c0_res = (collection_stop_tsc - m_maxTSC) - cx_res;
                        PWCollector_sample_t tps_sample = {m_id};
#if 0
                        PWCollector_sample_t tmp_wakeup_sample = {m_id};
                        create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, &tmp_wakeup_sample);
#endif
                        create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, NULL);
                        m_samples[C_STATE].push_back(tps_sample);
                    }
                } else {
                    /*
                     * Special case: this core never entered a c-state: we must account for the 'C0' time! We do
                     * so by inserting only a single 'boundary' sample.
                     */
                    {
                        /*
                         * Add a "ghost" wakeup sample at the end to adjust for
                         * C0+C1 time. This wakeup sample will have the following
                         * characteristics:
                         * 1. TSC == collection_stop_tsc;
                         * 2. C0 res == (collection_stop_tsc - collection_start_tsc) - 1 // we subtract one to account for the 1-cycle 'Cx' residency; see below
                         * 3. Cx res == 1 // We can't have a ZERO residency because that breaks AXE!!!
                         * 4. 'Which Cx' == {Lowest sleep state allowed by the user, defaults to APERF/C1}
                         * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                         * 6. Wakeup Cause == 'BOGUS'
                         * ---------------------------------------------------------------
                         * UPDATE: do this ONLY if the user asked for C-state samples!!!
                         * ---------------------------------------------------------------
                         */
                        pw_u64_t cx_res = 1;
                        pw_u64_t c0_res = (collection_stop_tsc - collection_start_tsc) - cx_res;
                        PWCollector_sample_t tps_sample = {m_id};
#if 0
                        PWCollector_sample_t tmp_wakeup_sample = {m_id};
                        create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, &tmp_wakeup_sample);
#endif
                        create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, NULL);
                        m_samples[C_STATE].push_back(tps_sample);
                    }
                }
                samples.insert(samples.end(), m_samples[C_STATE].begin(), m_samples[C_STATE].end());
            } else if (m_doesHaveMSRs) {
                /*
                 * User requested C-state collection, but for some reason no C-state samples
                 * were emitted. In this case, assume the thread/core/module/package was in C0
                 * 100% of the time. We do this by adding a "ghost" wakeup sample with the 
                 * following characteristics:
                 * 1. TSC == collection_stop_tsc;
                 * 2. C0 res == (collection_stop_tsc - collection_start_tsc) - 1 // we subtract one to account for the 1-cycle 'Cx' residency; see below
                 * 3. Cx res == 1 // We can't have a ZERO residency because that breaks AXE!!!
                 * 4. 'Which Cx' == {Lowest sleep state allowed by the user, defaults to APERF/C1}
                 * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                 * 6. Wakeup Cause == 'BOGUS'
                 * ---------------------------------------------------------------
                 * UPDATE: do this ONLY if the user asked for C-state samples!!!
                 * ---------------------------------------------------------------
                 */
                pw_u64_t cx_res = 1;
                pw_u64_t c0_res = (collection_stop_tsc - collection_start_tsc) - cx_res;
                PWCollector_sample_t tps_sample = {m_id};
#if 0
                PWCollector_sample_t tmp_wakeup_sample = {m_id};
                create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, &tmp_wakeup_sample);
#endif
                create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, NULL);
                m_samples[C_STATE].push_back(tps_sample);
                if (g_do_debugging) {
                    std::cerr << "Created new tps sample = " << tps_sample;
                }
                samples.insert(samples.end(), m_samples[C_STATE].begin(), m_samples[C_STATE].end());
            }
        } else if (m_doesHaveMSRs) {
            /*
             * Possible to have MSRs and not emit any C-state samples (e.g. pure "P", "S", or "D" state collection).
             * Handle later.
             */
        }
        for (unsigned i=0; i<m_children.size(); ++i) {
            m_children[i]->finalize_tps_samples(samples, collection_start_tsc, collection_stop_tsc);
        }
    };
#if 0
    void Processor::finalize_tps_samples(std::list <PWCollector_sample_t>& samples, const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc) {
        pw_u32_t collectionSwitches = pwr::WuData::instance()->getSystemInfo().m_collectionSwitches;
        if (m_samples[C_STATE].empty() == false) {
            assert(m_minTSC && m_maxTSC);
            // const PWCollector_sample_t& first_tps = m_samples[C_STATE].front(), &last_tps = m_samples[C_STATE].back();
            /*
             * Need a begin and (probably) an end BOUNDARY sample.
             */
            if (likely(collection_start_tsc < m_minTSC)) {
                /*
                 * Add a "ghost" wakeup sample at the beginning to adjust for
                 * C0+C1 time. This wakeup sample will have the following
                 * characteristics:
                 * 1. TSC == m_minTSC;
                 * 2. C0 res == (m_minTSC - collection_start_tsc) - 1 // we subtract one to account for the 1-cycle 'Cx' residency; see below
                 * 3. Cx res == 1 // We can't have a ZERO residency because that breaks AXE!!!
                 * 4. 'Which Cx' == {Lowest sleep state allowed by the user, defaults to APERF/C1}
                 * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                 * 6. Wakeup Cause == 'BOGUS'
                 * ---------------------------------------------------------------
                 * UPDATE: do this ONLY if the user asked for C-state samples!!!
                 * ---------------------------------------------------------------
                 */
                pw_u64_t cx_res = 1;
                pw_u64_t c0_res = (m_minTSC - collection_start_tsc) - cx_res;
                PWCollector_sample_t tmp_wakeup_sample = {m_id};
                PWCollector_sample_t tps_sample;
                create_c_state_sample_i(tps_sample, m_minTSC, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, &tmp_wakeup_sample);
                m_samples[C_STATE].push_front(tps_sample);
            }
            if (likely(collection_stop_tsc > m_maxTSC)) {
                /*
                 * Add a "ghost" wakeup sample at the end to adjust for
                 * C0+C1 time. This wakeup sample will have the following
                 * characteristics:
                 * 1. TSC == collection_stop_tsc;
                 * 2. C0 res == (collection_stop_tsc - m_maxTSC) - 1 // we subtract one to account for the 1-cycle 'Cx' residency; see below
                 * 3. Cx res == 1 // We can't have a ZERO residency because that breaks AXE!!!
                 * 4. 'Which Cx' == {Lowest sleep state allowed by the user, defaults to APERF/C1}
                 * 5. 'requested Cx' == <Don't Care, because the first sample will never have it's requested Cx printed anyway>
                 * 6. Wakeup Cause == 'BOGUS'
                 * ---------------------------------------------------------------
                 * UPDATE: do this ONLY if the user asked for C-state samples!!!
                 * ---------------------------------------------------------------
                 */
                pw_u64_t cx_res = 1;
                pw_u64_t c0_res = (collection_stop_tsc - m_maxTSC) - cx_res;
                PWCollector_sample_t tmp_wakeup_sample = {m_id};
                PWCollector_sample_t tps_sample;
                create_c_state_sample_i(tps_sample, collection_stop_tsc, APERF /* req_state */, APERF /* act_state */, c0_res, cx_res, &tmp_wakeup_sample);
                m_samples[C_STATE].push_back(tps_sample);
            }
            samples.insert(samples.end(), m_samples[C_STATE].begin(), m_samples[C_STATE].end());
        } else if (m_doesHaveMSRs) {
            /*
             * Possible to have MSRs and not emit any C-state samples (e.g. pure "P", "S", or "D" state collection).
             * Handle later.
             */
        }
        for (int i=0; i<m_children.size(); ++i) {
            m_children[i]->finalize_tps_samples(samples, collection_start_tsc, collection_stop_tsc);
        }
    };
#endif
    /*
     * 'Finalize' tpf samples: prune all TPF samples that don't fall within our calculated collection boundaries and introduce
     * new 'boundary' frequency samples, if required.
     * @samples: the list of P-State samples to output.
     * @collection_start_tsc, @collection_stop_tsc: calculated collection boundaries
     */
    void Processor::finalize_tpf_samples(std::list <PWCollector_sample_t>& samples, const pw_u64_t& collection_start_tsc, const pw_u64_t& collection_stop_tsc) {
        if (m_samples[P_STATE].empty() == false) {
            assert(collection_start_tsc && collection_stop_tsc);
            /*
             * First, calculate P-state transitions and prune those that didn't occur during the collection.
             */
            pw_u32_t collection_start_freq = 0, collection_stop_freq = 0;
            prune_tpf_samples_i(collection_start_tsc, collection_stop_tsc, collection_start_freq, collection_stop_freq);
            db_fprintf(stderr, "%s[%d]: collection start freq = %u, collection stop freq = %u\n", m_typeString.c_str(), m_id, collection_start_freq, collection_stop_freq);
            assert(collection_start_freq && collection_stop_freq);
            /*
             * OK, done pruning. Final step: add the two boundary samples for collection
             * START and STOP, respectively. Note that we might already have a sample with
             * TSC == collection_{start,stop}_tsc, in which case we merely mark it as a
             * boundary sample. On the other hand, if a boundary sample exists, but it's
             * TSC doesn't satisfy our {min,max} criteria, then we simply modify this boundary
             * so that the TSC's match.
             */
            if (m_samples[P_STATE].empty()) {
                PWCollector_sample_t tpf_sample;
                create_p_state_sample_i(tpf_sample, collection_start_tsc, collection_start_freq, 1); // "1" ==> START boundary
                m_samples[P_STATE].push_front(tpf_sample);
            } else if (m_samples[P_STATE].front().tsc > collection_start_tsc) {
                if (m_samples[P_STATE].front().p_sample.is_boundary_sample == 1) {
                    m_samples[P_STATE].front().tsc = collection_start_tsc;
                } else {
                    PWCollector_sample_t tpf_sample;
                    create_p_state_sample_i(tpf_sample, collection_start_tsc, collection_start_freq, 1); // "1" ==> START boundary
                    m_samples[P_STATE].push_front(tpf_sample);
                }
            } else {
                assert(m_samples[P_STATE].front().tsc == collection_start_tsc); // sanity!
                m_samples[P_STATE].front().p_sample.is_boundary_sample = 1;
            }
            assert(m_samples[P_STATE].empty() == false);
            if (m_samples[P_STATE].back().tsc < collection_stop_tsc) {
                if (m_samples[P_STATE].back().p_sample.is_boundary_sample < 2) {
                    PWCollector_sample_t tpf_sample;
                    create_p_state_sample_i(tpf_sample, collection_stop_tsc, collection_stop_freq, 2); // "2" ==> STOP boundary
                    m_samples[P_STATE].push_back(tpf_sample);
                } else {
                    m_samples[P_STATE].back().tsc = collection_stop_tsc;
                }
            } else {
                assert(m_samples[P_STATE].back().tsc == collection_stop_tsc); // sanity!
                m_samples[P_STATE].back().p_sample.is_boundary_sample = 2;
            }
            /*
             * OK, all adjustments made. Now add this cores P-state samples to the final list of P-state samples.
             */
            samples.insert(samples.end(), m_samples[P_STATE].begin(), m_samples[P_STATE].end());
        }
        Processor *child = NULL;
        FOR_EACH_CHILD_OF(this, child) {
            child->finalize_tpf_samples(samples, collection_start_tsc, collection_stop_tsc);
        }
    };

    /*
     * Helper struct: change 'cpuidx' field of every sample in a list.
     */
    struct SampleCpuidxChanger {
        int m_id;
        SampleCpuidxChanger(int i) : m_id(i) {};
        void operator()(PWCollector_sample_t& sample) {
            sample.cpuidx = m_id;
        };
    };
    /*
     * Add a list of samples of the given type to our output samples list.
     * @samples: the samples to append
     * @type: the type of samples contained in @sample
     */
    void Processor::append_samples(const std::list <PWCollector_sample_t>& samples, const sample_type_t& type) {
        if (m_wasSaltwell && type == C_STATE) {
            db_fprintf(stderr, "# C-state samples before merge = %u\n", (unsigned)m_samples[C_STATE].size());
            merge_c_state_samples(samples);
            db_fprintf(stderr, "# C-state samples after merge = %u\n", (unsigned)m_samples[C_STATE].size());
            return;
        } else if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel) && type == P_STATE) {
            db_fprintf(stderr, "# P-state samples before merge = %u\n", (unsigned)m_samples[P_STATE].size());
            merge_p_state_samples(samples);
            db_fprintf(stderr, "# P-state samples after merge = %u\n", (unsigned)m_samples[P_STATE].size());
            return;
        }
        std::list <PWCollector_sample_t>& my_samples = m_samples[type];
        my_samples.insert(my_samples.end(), samples.begin(), samples.end());
        /*
         * Make sure the samples being assigned to us all have the same cpuidx!
         */
        std::for_each(my_samples.begin(), my_samples.end(), SampleCpuidxChanger(m_id));
        /*
         * Samples are now unsorted. Sort them before doing anything else.
         */
        my_samples.sort();
        if (g_do_debugging) {
            std::copy(my_samples.begin(), my_samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
        }
    };
    /*
     * Merge package-level C-state samples into the cores. Currently only used by Saltwell cores.
     * @samples: the list of Package C-state samples.
     */
    void Processor::merge_c_state_samples(const std::list <PWCollector_sample_t>& samples)
    {
        /*
         * Special case for Saltwell C-state samples.
         */
        std::list<PWCollector_sample_t>::const_iterator pkg_iter = samples.begin();
        std::list <PWCollector_sample_t>& my_samples = m_samples[C_STATE];
        std::list<PWCollector_sample_t>::iterator curr_core_iter = my_samples.begin(), next_core_iter = curr_core_iter, prev_core_iter = curr_core_iter;

        if (unlikely(my_samples.empty())) {
            /*
             * Extremely unlikely! Should only happen if we explicitly disable HT!
             * In this case, just pull in all package samples and call them our own (the only
             * post-processing that's required is to convert the 'topology mask' in these
             * samples from 'PACKAGE_CPUIDX_MASK' to 'CORE_CPUIDX_MASK').
             */
            db_fprintf(stderr, "WARNING: [%s:%d] has empty C-state samples list?!\n", m_typeString.c_str(), m_id);
            my_samples = samples;
            for (curr_core_iter = my_samples.begin(); curr_core_iter != my_samples.end(); ++curr_core_iter) {
                // curr_core_iter->tsc &= ~PACKAGE_CPUIDX_MASK;
                /*
                 * First, strip away any previous mask. The mask is always the two most significant bits (i.e. bits 30, 31)
                 */
                curr_core_iter->cpuidx &= ~(3 << 30);
                /*
                 * Then add our id.
                 */
                curr_core_iter->cpuidx = m_id;
                /*
                 * And finally, add our own mask.
                 */
                add_topology_mask_to_cpuid_i(curr_core_iter->cpuidx);
            }
            return;
        }

        if (false && m_id == 0) {
            std::cerr << "Pkg samples...\n";
            std::copy(samples.begin(), samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
            std::cerr << "My samples...\n";
            std::copy(my_samples.begin(), my_samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
            assert(false);
        }
        /*
         * Special case the first package sample here.
         */
        {
            pw_u32_t prev_state = pkg_iter->c_sample.prev_state, prev_act = GET_ACT_FROM_PREV(prev_state);
            //pw_u32_t prev_req = GET_REQ_FROM_PREV(prev_state);
            pw_u64_t prev_cx = RES_COUNT(pkg_iter->c_sample, prev_act), prev_c0 = RES_COUNT(pkg_iter->c_sample, MPERF);
            pw_u64_t prev_tsc = pkg_iter->tsc - prev_cx - prev_c0; // The TSC of the last time the core/package entered idle.
            if (pkg_iter->tsc < curr_core_iter->tsc) {
                // Add 'pkg' to HEAD of core list;
                PWCollector_sample_t tps_sample = *pkg_iter;
                tps_sample.cpuidx = m_id;
                db_fprintf(stderr, "Before: curr = %llu ", curr_core_iter->tsc);
                my_samples.push_front(tps_sample);
                db_fprintf(stderr, " After: curr = %llu\n", curr_core_iter->tsc);
                // ++pkg_iter;
                while ((++pkg_iter)->tsc < curr_core_iter->tsc) {
                    tps_sample = *pkg_iter;
                    tps_sample.cpuidx = m_id;
                    db_fprintf(stderr, "Before: curr = %llu ", curr_core_iter->tsc);
                    // my_samples.push_front(tps_sample);
                    my_samples.insert(curr_core_iter, tps_sample); // inserts BEFORE 'curr'
                    db_fprintf(stderr, " After: curr = %llu\n", curr_core_iter->tsc);
                }
            } else if (prev_tsc >= curr_core_iter->tsc) {
                /*
                 * First package C-state entry occurs AFTER the first core C-state
                 * entry. Note that we could have multiple core C-state samples before
                 * the first package C-state entry. Delete all but the last such core C-state
                 * sample.
                 */
                db_fprintf(stderr, "Starting: Curr = %llu\n", curr_core_iter->tsc);
                for (; next_core_iter != my_samples.end() && next_core_iter->tsc <= prev_tsc; curr_core_iter = next_core_iter, ++next_core_iter);
                /*
                 * Delete all samples upto 'prev_tsc'
                 */
                db_fprintf(stderr, "Prev_tsc = %llu, Curr = %llu, Next = %llu\n", prev_tsc, curr_core_iter->tsc, next_core_iter->tsc);
                if (prev_tsc == curr_core_iter->tsc) {
                    // Nothing to do
                    // prev_core_iter = curr_core_iter; curr_core_iter = next_core_iter; ++next_core_iter;
                    prev_core_iter = curr_core_iter = next_core_iter;
                } else {
                    /*
                     * Package entered (and exitted) idle after 'curr_core_iter->tsc', but before 'next_core_iter->tsc'.
                     * To handle this case, we add a new 'C1*' sample between the two. The TSC value of this new sample
                     * will be 'prev_tsc', the C0 residency will be zero and the C1 residency will be (prev_tsc - curr_core_iter->tsc).
                     */
                    /*
                     * Step 1: add a new C1* sample.
                     */
                    {
                        PWCollector_sample_t tps_sample;
                        pw_u64_t __tsc = prev_tsc, __cx_res = __tsc - curr_core_iter->tsc, __c0_res = 0x0; // Make sure all time is attributed to 'C1'
                        pw_u32_t __state = next_core_iter->c_sample.prev_state, __req_state = GET_REQ_FROM_PREV(__state);
                        //pw_u32_t __act_state = GET_ACT_FROM_PREV(__state);
                        create_c_state_sample_i(tps_sample, __tsc, (c_state_t)__req_state, APERF /* act state */, __c0_res, __cx_res, NULL /* wu cause */);
                        tps_sample.c_sample.break_type = PW_BREAK_TYPE_N;
                        my_samples.insert(next_core_iter, tps_sample); // inserts BEFORE 'next'
                        if (g_do_debugging) {
                            std::cerr << "Created new TPS sample: " << tps_sample;
                        }
                        // curr_core_iter = next_core_iter; ++next_core_iter;
                        prev_core_iter = curr_core_iter = next_core_iter; // ++next_core_iter;
                        // prev_core_iter = curr_core_iter; ++curr_core_iter; // "curr" now points to "tps_sample"
                        if (g_do_debugging) {
                            std::cerr << "New curr = " << *curr_core_iter;
                            std::cerr << "New next = " << *next_core_iter;
                        }
                    }
                    /*
                     * Step 2: modify the 'next' sample to transform it from a core C-state sample to a Pkg C-state sample.
                     * Step 3: add a new C1 * sample.
                     *
                     * BOTH done in for loop, below
                     */
                    prev_core_iter = curr_core_iter;
                }
            }
        }

        /*
         * LOOP INVARIANT: 'curr_core_iter'->tsc <= 'pkg_iter'->tsc at the start of each iteration.
         */
        for (; pkg_iter != samples.end() && next_core_iter != my_samples.end(); prev_core_iter = curr_core_iter, curr_core_iter = next_core_iter, ++pkg_iter) {
            /*
             * First, transfer 'Cx' residencies and wu causes from 'pkg' to 'curr_core'
             * Note: check corner cases e.g. is the 'curr_core' still valid?!
             */
            assert(curr_core_iter != my_samples.end());
            if (prev_core_iter->tsc > pkg_iter->tsc) {
                db_fprintf(stderr, "ERROR: prev_core_iter->tsc == %llu, pkg_iter->tsc == %llu\n", prev_core_iter->tsc, pkg_iter->tsc);
            }
            assert(prev_core_iter->tsc <= pkg_iter->tsc);
            if (pkg_iter->tsc < curr_core_iter->tsc) {
                /*
                 * This usually happens because we've dropped core samples (buffer overflow on driver side).
                 * Insert 'pkg_iter' BEFORE curr_core and move to the next package sample.
                 */
                // Add new sample after 'curr' and before 'next'
                PWCollector_sample_t tps_sample;
                pw_u32_t pkg_state = pkg_iter->c_sample.prev_state, pkg_req = GET_REQ_FROM_PREV(pkg_state), pkg_act = GET_ACT_FROM_PREV(pkg_state);
                pw_u64_t cx_res = RES_COUNT(pkg_iter->c_sample, pkg_act), c0_res = 0;
                pw_u64_t tsc = pkg_iter->tsc, tsc_delta = pkg_iter->tsc - prev_core_iter->tsc;
                // assert(cx_res);
                if (!cx_res) {
                    fprintf(stderr, "WARNING: zero Cx res for req_state = %u, act_state = %u, prev_state = %u at pkg tsc = %llu\n", pkg_req, pkg_act, pkg_state, tsc);
                    db_abort("Warning: possible errors with Cx residency computation!\n");
                }
                if (tsc_delta < cx_res) {
                    fprintf(stderr, "WARNING[1]: at pkg tsc = %llu, tsc_delta = %llu, delta_cx = %llu\n", pkg_iter->tsc, tsc_delta, cx_res);
                    cx_res = tsc_delta - 1;
                    // ++s_num_pkg_cx_overcounts;
                }
                c0_res = tsc_delta - cx_res;

                create_c_state_sample_i(tps_sample, tsc, (c_state_t)pkg_req/* req state */, (c_state_t)pkg_act/* act state */, c0_res, cx_res, NULL /* wu cause */);

                tps_sample.c_sample.break_type = pkg_iter->c_sample.break_type;
                /*
                tps_sample.c_sample.pid = pkg_iter->e_sample.data[0];
                tps_sample.c_sample.tid = pkg_iter->e_sample.data[1];
                tps_sample.c_sample.c_data = pkg_iter->e_sample.data[2];
                */
                tps_sample.c_sample.pid = pkg_iter->c_sample.pid;
                tps_sample.c_sample.tid = pkg_iter->c_sample.tid;
                tps_sample.c_sample.c_data = pkg_iter->c_sample.c_data;
                my_samples.insert(curr_core_iter, tps_sample); // inserts BEFORE 'curr'
                db_fprintf(stderr, "prev = %llu, curr = %llu, next = %llu, pkg = %llu\n", prev_core_iter->tsc, curr_core_iter->tsc, next_core_iter->tsc, pkg_iter->tsc);

                curr_core_iter = prev_core_iter;
                ++curr_core_iter; // should now point to the newly inserted sample

                continue;
            }
            assert(curr_core_iter->tsc <= pkg_iter->tsc);
            db_fprintf(stderr, "%llu\n", pkg_iter->tsc);
            {
                /*
                 * Core entered idle BEFORE the package entered idle. To account for this, we assign the
                 * time difference between the core and package idle enter events as C1 time for this core.
                 * (1) Start off by assigning the 'Cx' res in 'next' to 'curr'. Note that we'll have to recalculate
                 * the 'C0' res for 'curr' in the process.
                 */
                pw_u32_t prev_state = pkg_iter->c_sample.prev_state, prev_act = GET_ACT_FROM_PREV(prev_state);
                //pw_u32_t prev_req = GET_REQ_FROM_PREV(prev_state);
                pw_u64_t prev_cx = RES_COUNT(pkg_iter->c_sample, prev_act), prev_c0 = RES_COUNT(pkg_iter->c_sample, MPERF);
                /*
                 * The C1* sample will have the same 'prev_state' and 'Cx' res as 'next'...
                 */
                curr_core_iter->c_sample.prev_state = prev_state;
                RES_COUNT(curr_core_iter->c_sample, prev_act) = prev_cx;
                /*
                 * ...but will have a new 'C0' res.
                 */
                pw_u64_t prev_tsc = pkg_iter->tsc - prev_cx - prev_c0; // The TSC of the last time the core/package entered idle.
                pw_u64_t delta_tsc = (curr_core_iter->tsc - prev_tsc);
                /*
                 * Note: in some cases, Delta-Cx >= Delta-TSC. I suspect this is a H/W issue (inaccurate clock?).
                 * For now, we set the MPERF to one in these cases.
                 */
                if (delta_tsc < prev_cx) {
                    fprintf(stderr, "WARNING[2] at pkg tsc = %llu, delta_Tsc = %llu, delta_cx = %llu\n", pkg_iter->tsc, delta_tsc, prev_cx);
                    prev_cx = delta_tsc - 1;
                    // ++s_num_pkg_cx_overcounts;
                }
                pw_u64_t new_curr_c0_res = delta_tsc - prev_cx; // C0 == TSC-delta - Cx-residency
                RES_COUNT(curr_core_iter->c_sample, MPERF) = new_curr_c0_res;
                // RES_COUNT(curr_core_iter->c_sample, prev_act) = prev_cx;
                /*
                 * (2) Make sure we erase any previous residency from 'curr'!
                 */
                RES_COUNT(curr_core_iter->c_sample, APERF) = 0x0;
                /*
                 * (3) And finally, assign the wakeup cause from 'next' to the C1* sample.
                 */
                curr_core_iter->c_sample.break_type = pkg_iter->c_sample.break_type;
                curr_core_iter->c_sample.pid = pkg_iter->c_sample.pid;
                curr_core_iter->c_sample.tid = pkg_iter->c_sample.tid;
                curr_core_iter->c_sample.c_data = pkg_iter->c_sample.c_data;
                /*
                 * UPDATE: don't forget to update the 'wakeup_cpu' field!
                 */
                curr_core_iter->c_sample.tps_epoch = pkg_iter->c_sample.tps_epoch;

                db_fprintf(stderr, "OK: assigned cx from TSC = %llu to TSC = %llu, prev_tsc = %llu, prev_cx = %llu, next_tsc = %llu\n", pkg_iter->tsc, curr_core_iter->tsc, prev_tsc, prev_cx, next_core_iter->tsc);

                /*
                 * It is possible that 'curr'->tsc == 'pkg'->tsc. In this case, we're done with the current iteration.
                 */
                if (unlikely(curr_core_iter->tsc == pkg_iter->tsc)) {
                    db_fprintf(stderr, "OK: CURR TSC = %llu is EQUAL; nothing to do!\n", pkg_iter->tsc);
                    ++next_core_iter;
                    continue;
                }
            }
            /*
             * Then, find 'pkg' position within core list.
             * UPDATE: also handle converting the wakeup types to 'BREAK_TYPE_N' and converting
             * C0 residencies to zero.
             */
            {
                for (++next_core_iter; next_core_iter != my_samples.end() && next_core_iter->tsc < pkg_iter->tsc; curr_core_iter = next_core_iter, ++next_core_iter) {
                    if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                        pw_u32_t next_core_prev_state = next_core_iter->c_sample.prev_state, next_core_req = GET_REQ_FROM_PREV(next_core_prev_state), next_core_act = GET_ACT_FROM_PREV(next_core_prev_state);
                        assert(next_core_act == APERF); // act state MUST be C1!!!
                        if (next_core_req == APERF) {
                            if (g_do_debugging) {
                                std::cerr << "ACTUAL APERF sample: " << *next_core_iter;
                            }
                            continue;
                        } else if (next_core_iter->c_sample.break_type == PW_BREAK_TYPE_A) {
                            if (g_do_debugging) {
                                std::cerr << "ABORT sample: " << *next_core_iter;
                            }
                            continue;
                        }
                        // next_core_iter->c_sample.break_type = PW_BREAK_TYPE_N;
                        /*
                         * Need to patch up the residencies -- these should contain a zero C0 residency and
                         * a non-zero C1 residency.
                         */
                        /*
                        pw_u64_t tsc_delta = (next_core_iter->tsc - curr_core_iter->tsc);
                        RES_COUNT(next_core_iter->c_sample, APERF) = tsc_delta; RES_COUNT(next_core_iter->c_sample, MPERF) = 0x0;
                        */
                        pw_u64_t __tsc_delta = (next_core_iter->tsc - curr_core_iter->tsc);
                        pw_u64_t __c1_res = RES_COUNT(next_core_iter->c_sample, APERF), __c0_res = 0x0;
                        if (__c1_res == 0x1) {
                            fprintf(stderr, "POSSIBLE ABORT at tsc = %llu!\n", next_core_iter->tsc);
                        }
                        if (__tsc_delta < __c1_res) {
                            db_fprintf(stderr, "WARNING: at tsc = %llu, delta tsc = %llu, delta aperf = %llu\n", next_core_iter->tsc, __tsc_delta, __c1_res);
                            __c1_res = __tsc_delta - 1;
                            RES_COUNT(next_core_iter->c_sample, APERF) = __c1_res;
                        }
                        __c0_res = __tsc_delta - __c1_res; // C0 == TSC-delta - Cx-residency
                        RES_COUNT(next_core_iter->c_sample, MPERF) = __c0_res;
                    }
                }
            }
            /*
             * Finally, either modify an existing core sample or insert a new one to depict the package c-state entry.
             */
            if (next_core_iter != my_samples.end()) {
                /*
                 * Found!
                 */
                if (next_core_iter->tsc == pkg_iter->tsc) {
                    // Modify 'next_core_iter' in place: recalculate APERF, MPERF.
                    // UPDATE: not required to modify ANYTHING!
                    curr_core_iter = next_core_iter;
                    ++next_core_iter; // Make sure we set 'curr' to the next sample after this one!
                    db_fprintf(stderr, "OK: TSC = %llu is EQUAL; nothing to do! Next core tsc = %llu\n", pkg_iter->tsc, next_core_iter->tsc);
                    if (next_core_iter != my_samples.end()) {
                        pw_u64_t __tsc_delta = next_core_iter->tsc - pkg_iter->tsc, __c0_res = RES_COUNT(next_core_iter->c_sample, MPERF), __cx_res = RES_COUNT(next_core_iter->c_sample, APERF);
                        assert(GET_ACT_FROM_PREV(next_core_iter->c_sample.prev_state) == APERF); // sanity!
                        if (__tsc_delta != (__c0_res + __cx_res)) {
                            fprintf(stderr, "NOT equal at TSC = %llu\n", pkg_iter->tsc);
                        }
                    }
                } else {
                    // Add new sample after 'curr' and before 'next'
                    PWCollector_sample_t tps_sample;
                    pw_u64_t tsc = pkg_iter->tsc;
                    pw_u32_t __prev_state = next_core_iter->c_sample.prev_state, __req_state = GET_REQ_FROM_PREV(__prev_state);
                    // pw_u32_t __act_state = GET_ACT_FROM_PREV(__prev_state);
                    pw_u64_t c0_res = 0x0, cx_res = pkg_iter->tsc - curr_core_iter->tsc; // Make sure all time is attributed to 'C1'
                    create_c_state_sample_i(tps_sample, tsc, (c_state_t)__req_state /* req state */, APERF /* act state */, c0_res, cx_res, NULL /* wu cause */);
                    db_fprintf(stderr, "OK: adding TSC = %llu between %llu and %llu\n", pkg_iter->tsc, curr_core_iter->tsc, next_core_iter->tsc);
                    tps_sample.c_sample.break_type = PW_BREAK_TYPE_N;
                    my_samples.insert(next_core_iter, tps_sample); // inserts BEFORE 'next'
                    ++curr_core_iter;
                    /*
                     * We've probably messed up the 'TSC-delta - C0-delta' calculations for 'next_core_iter'. Ordinarilly this
                     * isn't a problem (because the next 'pkg_iter' insertion will automatically adjust that for us). However,
                     * if this is the LAST pkg sample then we'll exit the loop after this iteration. The end result is: the
                     * TSC-delta between the newly inserted sample (i.e. 'tps_sample', above) and the 'next_core_iter' sample
                     * will be LESS than the 'C0+CX' residency of 'next_core_iter'.
                     * To avoid this, we proactively patch up that difference here.
                     */
                    {
                        pw_u64_t __tsc_delta = next_core_iter->tsc - tsc, __c0_res = RES_COUNT(next_core_iter->c_sample, MPERF);
                        pw_u64_t __cx_res = 0x0;
                        assert(GET_ACT_FROM_PREV(next_core_iter->c_sample.prev_state) == APERF); // sanity!
                        if (unlikely(__tsc_delta < __c0_res)) {
                            db_fprintf(stderr, "C1 warning in patch-up! tsc-delta = %llu, c0-res = %llu\n", __tsc_delta, __c0_res);
                            __c0_res = __tsc_delta;
                        }
                        __cx_res = __tsc_delta - __c0_res;
                        RES_COUNT(next_core_iter->c_sample, MPERF) = __c0_res; RES_COUNT(next_core_iter->c_sample, APERF) = __cx_res;
                    }
                }
            } else {
                /*
                 * Not found! Append to end of core list.
                 * Note that all future pkg samples need to be appended
                 * to the end of the core list.
                 * ------------------------------------------------------------------------
                 * Sanity: can we have any further pkg samples if we've reached
                 * the end of the core list?!
                 * Ans: YES, because a core sample is only created when we see the next
                 * core TPS, which might not happen if the last TPS was on the other core.
                 * ------------------------------------------------------------------------
                 */
                PWCollector_sample_t tps_sample;
                pw_u64_t tsc = pkg_iter->tsc;
                pw_u64_t c0_res = 0x0, cx_res = pkg_iter->tsc - curr_core_iter->tsc; // Make sure all time is attributed to 'C1'
                c_break_type_t __break_type = PW_BREAK_TYPE_N;
                c_state_t __req_state = APERF, __act_state = APERF;
                if (PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    /*
                     * The ONLY reason we can get here is if this is the LAST package sample and we've seen only one
                     * thread entering 'mwait'. Special case that here.
                     */
                    c0_res = cx_res; cx_res = 0;
                    __break_type = PW_BREAK_TYPE_A;
                    __req_state = (c_state_t)GET_REQ_FROM_PREV(curr_core_iter->c_sample.prev_state);
                    db_fprintf(stderr, "WARNING: MFD had possible 'N' break type at pkg TSC = %llu, core TSC = %llu: RESET TO ABORT!\n", pkg_iter->tsc, curr_core_iter->tsc);
                }
                create_c_state_sample_i(tps_sample, tsc, __req_state, __act_state, c0_res, cx_res, NULL /* wu cause */);
                db_fprintf(stderr, "OK: appending TSC = %llu to end of list!\n", pkg_iter->tsc);
                tps_sample.c_sample.break_type = __break_type;
                my_samples.push_back(tps_sample);
                // next_core_iter = curr_core_iter; // Make sure the "curr=next" in the loop header doesn't cause us to overflow!
            }
        }
        if (pkg_iter != samples.end()) {
            // assert(PW_IS_MFD(pwr::WuData::instance()->getSystemInfo().m_cpuModel) == false);
            if (g_do_debugging) {
                std::cerr << "pkg_iter = " << *pkg_iter;
            }
            curr_core_iter = --next_core_iter;
            my_samples.insert(my_samples.end(), pkg_iter, samples.end());
            next_core_iter = curr_core_iter;
#if 1
            for (++next_core_iter; next_core_iter != my_samples.end(); ++next_core_iter) {
                /*
                 * We append all such samples to the end of our list with minimal changes
                 * (change the 'cpuidx' to an appropriate value; make sure we recalculate the 
                 * first sample's C0 residency etc.).
                 */
                if (g_do_debugging) {
                    std::cerr << "Next core before = " << *next_core_iter;
                }
                /*
                 * First, strip away any previous mask. The mask is always the two most significant bits (i.e. bits 30, 31)
                 */
                next_core_iter->cpuidx &= ~(3 << 30);
                /*
                 * Then add our id.
                 */
                next_core_iter->cpuidx = m_id;
                /*
                 * And finally, add our own mask.
                 */
                add_topology_mask_to_cpuid_i(next_core_iter->cpuidx);
                if (g_do_debugging) {
                    std::cerr << "Next core after = " << *next_core_iter;
                }
            }
#endif
            if (false) {
                std::copy(my_samples.begin(), my_samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
                assert(false);
            }
        }
#if 0
        /*
         * We need to do some book-keeping for CLTP samples.
         */
        {
            if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                for (curr_core_iter = my_samples.begin(); curr_core_iter != my_samples.end(); ++curr_core_iter) {
                    pw_u32_t __prev_state = curr_core_iter->c_sample.prev_state, __req_state = GET_REQ_FROM_PREV(__prev_state), __act_state = GET_ACT_FROM_PREV(__prev_state);
                    if (__act_state == APERF) {
                        pw_u64_t& __c1_res = RES_COUNT(curr_core_iter->c_sample, APERF); 
                        if (__req_state > APERF) {
                            switch (curr_core_iter->c_sample.break_type) {
                                // case PW_BREAK_TYPE_N:
                                case PW_BREAK_TYPE_A:
                                    break;
                                default:
                                    /*
                                     * We need to tell AXE of 'CI1/C1*' samples. We do so by setting the highest bit in 
                                     * the 'C1' res count (we assume C1 residency will always be less than 0x7fffffffffffffff ticks).
                                     */
                                    SET_C1I_FLAG(__c1_res);
                                    db_fprintf(stderr, "%llu -> %llu, %s\n", curr_core_iter->tsc, __c1_res, GET_BOOL_STRING(IS_C1I_FLAG_SET(RES_COUNT(curr_core_iter->c_sample, APERF))));
                                    // assert(false);
                                    break;
                            }
                        } else {
                            /*
                             * A valid 'C1' sample -- make sure we reset the MSB!
                             */
                            RESET_C1I_FLAG(__c1_res);
                        }
                    }
                }
            }
        }
#endif
        if (false) {
            std::cerr << "Merged samples...\n";
            std::copy(my_samples.begin(), my_samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
            if (m_id == 0) {
                assert(false);
            }
        }
    };
    /*
     * Merge package-level 'P-state' samples into the cores. This is a CLTP-specific hack, necessitated
     * due to the P-state problems associated with a common voltage rail.
     */
    void Processor::merge_p_state_samples(const std::list <PWCollector_sample_t>& samples) {
        if (samples.empty()) {
            return;
        }
        std::list <PWCollector_sample_t>& my_samples = m_samples[P_STATE];
        assert(my_samples.empty());
        /*
         * We need to:
         * 1. Extract all samples that belong to us (use the 'cpuidx' field to determine if a sample belongs to us) and
         * 2. Add our own 'topology mask' to these samples.
         */
        for (std::list<PWCollector_sample_t>::const_iterator pkg_iter = samples.begin(); pkg_iter != samples.end(); ++pkg_iter) {
            if (GET_CORE_GIVEN_LCPU(pkg_iter->cpuidx) != m_id) {
                continue;
            }
            PWCollector_sample_t tpf_sample = *pkg_iter;
            /*
             * First, add our id.
             */
            tpf_sample.cpuidx = m_id;
            /*
             * And finally, add our own mask.
             */
            add_topology_mask_to_cpuid_i(tpf_sample.cpuidx);
            my_samples.push_back(tpf_sample);
        }
    };

    /*
     * Push all samples to the cores.
     */
    void Processor::push_samples_to_cores() {
        push_samples_to_cores_i();
    };
    /*
     * Add a child node.
     * @child: the node to be added.
     */
    void Processor::add_child(Processor *child) {
        child->m_parent = this;
        m_children.push_back(child);
    };
    /*
     * Set the supported MSRs for this particular node. Note that this
     * usually applies only to cores and/or packages, and not to threads.
     * @supported_msrs: the list of supported MSRs
     */
    void Processor::set_msrs(const std::vector<int>& supported_msrs) {
        for (unsigned i=0; i<supported_msrs.size(); ++i) {
            if (supported_msrs[i] != -1) {
                m_cStateMSRSet[i] = 0;
                if (i > 0) {
                    m_doesHaveMSRs = true;
                }
            }
        }
    };
    /*
     * Debugging: traverse the topology tree and print out some debugging
     * information for each node.
     */
    void Processor::dfs_traverse(int level) const {
        for (int i=0; i<level; ++i) {
            db_fprintf(stderr, "\t");
        }
        dump();
        for (std::vector<Processor *>::const_iterator citer = m_children.begin(); citer != m_children.end(); ++citer) {
            (*citer)->dfs_traverse(level+1);
        }
    };

    std::string Processor::name(void) const {
        std::stringstream stream;
        stream << m_typeString << "[" << m_id << "]";

        return stream.str();
    };

    class Package : public Processor {
        public:
            Package(int id) : Processor("Package", id) {
                m_parent = NULL;
            };
        private:
            /*
             * INTERNAL API:
             * Package-specific callback for 'idle'-related samples (TPS, IRQ etc.)
             */
            int handle_idle_sample_i(const PWCollector_sample_t *sample) {
                /*
                 * We've received a sample that could potentially alter our DFA state.
                 */
                update_non_leaf_dfa_state_i(sample);
                /*
                 * Did we confirm a previous idle? If so, notify our children to let them
                 * take any required action. We need this only to handle C1 aborts in Saltwell.
                 */
                if (m_parentIdleSnapshotIdx >= 0) {
                    /*
                     * We'll need to notify our children that we entered idle.
                     */
                    notify_children_of_idle_i(&m_idleSnapshots[m_parentIdleSnapshotIdx]);
                    m_parentIdleSnapshotIdx = -1;
                }
                return PW_SUCCESS;
            };
            /*
             * INTERNAL API:
             * Package-specific callback for frequency samples. Only required
             * for CLTP.
             */
            /*
#define CLTP_CRITICAL_FREQUENCY() (1600000)
*/
#define CLTP_CRITICAL_FREQUENCY_MULTIPLIER() (12)
#define CLTP_CRITICAL_FREQUENCY() ( GET_INT_FROM_FLOAT((float)CLTP_CRITICAL_FREQUENCY_MULTIPLIER() * pwr::WuData::instance()->getSystemInfo().m_busClockFreqMHz) * 1000 )
            int handle_freq_sample_i(const PWCollector_sample_t *sample) {
                pw_u32_t oldActFreq = m_currActFreq;
                int thread_id = sample->cpuidx, core_id = GET_CORE_GIVEN_LCPU(thread_id);

                // update_non_leaf_freq_state_i(sample);

                /*
                 * Only CLTP packages need to handle P-state samples.
                 */
                if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    m_samples[P_STATE].push_back(*sample);
                    /*
                     * Infer the 'actual' frequency this node is executing at. Note that
                     * this is completely hypothetical for a package node; we use this as
                     * a convenience to decide when to generate artifical TPF samples
                     * for CTP.
                     */
                    {
                        Processor *__child = NULL;
                        pw_u32_t __freq = 0x0;
                        FOR_EACH_CHILD_OF(this, __child) {
                            // __freq = std::max(__freq, __child->get_curr_act_freq());
                            __freq = std::max(__freq, __child->get_prime_freq());
                        }
                        m_currActFreq = __freq;
                    }
#if 1
                    if (likely(sample->p_sample.is_boundary_sample != 1)) { // "1" ==> BEGIN boundary
                        pw_u64_t tsc_new = 0x0;
                        pw_u32_t req_freq_new = 0x0, act_freq_new = 0x0;
                        int cpuidx_new = -1;
                        bool should_create_new_sample = false;
                        int __tmp1 = GET_INT_FROM_FLOAT((float)CLTP_CRITICAL_FREQUENCY_MULTIPLIER() * pwr::WuData::instance()->getSystemInfo().m_busClockFreqMHz);
                        if (oldActFreq < CLTP_CRITICAL_FREQUENCY() && m_currActFreq >= CLTP_CRITICAL_FREQUENCY()) {
                            /*
                             * The package thinks it has gone critical i.e. one core has started to execute above
                             * the 1200 MHz barrier. This means the frequency on the other core will be scaled up
                             * to 1200 MHz, and this scaling will be involuntary i.e. independent of any OS requests
                             * on that core. We handle this situation by generating an 'artifical' TPF sample for that
                             * other core.
                             */
                            Processor *other_core = get_child_given_id_i(1-core_id);
                            pw_u32_t prime_freq = other_core->get_prime_freq();
                            // cpuidx_new = 1 - core_id;
                            cpuidx_new = other_core->get_first_leaf_of();
                            req_freq_new = other_core->get_curr_req_freq();
                            act_freq_new = prime_freq;
                            tsc_new = sample->tsc;
                            should_create_new_sample = true;
                            db_fprintf(stderr, "PKG went critical at TSC = %llu. Offending thread = %d, core = %d, other_core = %d (%p), f-prime = %u\n", sample->tsc, thread_id, core_id, (1 - core_id), other_core, prime_freq);
                        } else if (oldActFreq >= CLTP_CRITICAL_FREQUENCY() && m_currActFreq < CLTP_CRITICAL_FREQUENCY()) {
                            Processor *other_core = get_child_given_id_i(1-core_id);
                            pw_u32_t prime_freq = other_core->get_prime_freq();
                            // cpuidx_new = 1 - core_id;
                            cpuidx_new = other_core->get_first_leaf_of();
                            req_freq_new = other_core->get_curr_req_freq();
                            act_freq_new = CLTP_CRITICAL_FREQUENCY();
                            tsc_new = sample->tsc;
                            should_create_new_sample = true;
                            db_fprintf(stderr, "PKG went subcritical at TSC = %llu. Offending thread = %d, core = %d, other_core = %d (%p), f-prime = %u\n", sample->tsc, thread_id, core_id, (1 - core_id), other_core, prime_freq);
                        } else {
                            db_fprintf(stderr, "PKG status quo at TSC = %llu. oldAct = %u, newAct = %u\n", sample->tsc, oldActFreq, m_currActFreq);
                        }
                        if (should_create_new_sample) { // (req_freq_new)
                            PWCollector_sample_t __tpf_sample = {cpuidx_new, P_STATE, 0x0 /* sample_len, don't care */, tsc_new};
                            __tpf_sample.p_sample.prev_req_frequency = req_freq_new;
                            __tpf_sample.p_sample.frequency = act_freq_new;
                            if (g_do_debugging) {
                                std::cerr << __tpf_sample;
                            }
                            assert(__tpf_sample.tsc == m_samples[P_STATE].back().tsc);
                            m_samples[P_STATE].push_back(__tpf_sample);
                            dump_tree_freq_state_i(0);
                        }
                    }
#endif
                }
                return PW_SUCCESS;
            };
            /*
             * INTERNAL API:
             * Push all package samples to cores.
             */
            void push_samples_to_cores_i() {
                Processor *child = NULL;
                /*
                 * We do NOT push ALL sample types to cores. Currently, we ONLY push the
                 * following sample types from packages to cores:
                 * 1. C-state
                 * 2. P-state (CLTP-specific)
                 * All other samples remain at the package level.
                 */
                for (int i=C_STATE; i<=P_STATE; ++i) {
                    std::list <PWCollector_sample_t>& samples = m_samples[(sample_type_t)i];
                    if (samples.empty()) {
                        continue;
                    }
                    FOR_EACH_CHILD_OF(this, child) {
                        child->append_samples(samples, (sample_type_t)i);
                    }
                    if (i > C_STATE) {
                        samples.clear();
                    }
                }
                /*
                std::list <PWCollector_sample_t>& samples = m_samples[C_STATE];
                if (samples.empty()) {
                    return;
                }
                FOR_EACH_CHILD_OF(this, child) {
                    child->append_samples(samples, C_STATE);
                }
                */
            };

            /*
             * INTERNAL API:
             * Function to add a mask to individual samples to identify them as being
             * package-specific.
             * @cpuid: the field to which a mask is added.
             */
            void add_topology_mask_to_cpuid_i(pw_u32_t& cpuid) {
                cpuid |= PACKAGE_CPUIDX_MASK;
            };
            /*
             * INTERNAL API:
             * Was the previous Package idle enter valid?
             * @req_state: state requested by OS on PREVIOUS idle-enter.
             * @act_state: state granted by H/W on PREVIOUS idle-enter
             *
             * @returns: true if previous idle was valid, false otherwise
             */
            bool is_valid_idle_i(c_state_t req_state, c_state_t act_state) {
                /*
                 * Package-specific C1 check: there are NO Package C1 states on ANY platform!
                 * This means that 'act_state' can NEVER be 'C1' on a package level! Also,
                 * a case where the Package MSRs didn't count, but the package state resolved
                 * to 'C1' is NOT a valid package idle (instead, we treat it as both cores
                 * entering 'C1' independently, but that is auto-handled by the cores themselves).
                 */
                return (act_state > MPERF);
            };

            void handle_abort_i(const PWCollector_sample_t *sample) {
                /*
                 * Basic algo:
                 * 1. Find core that triggered the abort. This is the core that last tried to enter mwait.
                 * 2. Reset snapshot for core from 1.
                 * 3. Iterate through all other cores:
                 * a. If any core's 'm_currDfaState' is NOT 'DFA_C_X' then reset it's snapshot.
                 *
                 * UPDATE: we do NOT consider a req_state == MPERF or APERF to be an abort (MPERF because
                 * that indicates an incomplete snapshot; APERF because that means the cores have entered C1/C1I
                 * on their own and must not be aborted out).
                 */
                IdleSnapshot& snapshot = m_idleSnapshots.back();
                if (snapshot.req_state == MPERF || snapshot.req_state == APERF) {
                    return;
                }
                pw_u64_t abort_tsc = snapshot.tps->tsc;
                int abort_cpuidx = snapshot.tps->cpuidx;
                int abort_coreid = GET_CORE_GIVEN_LCPU(abort_cpuidx), threadid = -1;
                Processor *child = NULL;

                if (g_do_debugging) {
                    std::cerr << "Abort for idle snapshot at tsc = " << sample->tsc << "\n";
                    std::cerr << "Snapshot before reset ..." << snapshot;
                    std::cerr << "Thread id = " << abort_cpuidx << ", core id = " << abort_coreid << "\n";
                }
                for_each_thread_in_core(abort_coreid, threadid) {
                    db_fprintf(stderr, "Core id = %d, thread id = %d\n", abort_coreid, threadid);
                    snapshot.threadSnapshots[threadid] = s_emptyThread;
                }
                if (false) {
                    return;
                }
                FOR_EACH_CHILD_OF(this, child) {
                    notify_child_of_abort_i(child, abort_tsc);
                    if (child->get_id() != abort_coreid) {
                        int __tmp_coreid = child->get_id();
                        DFA_state childState = child->get_current_dfa_state();
                        if (childState != DFA_C_X) {
                            db_fprintf(stderr, "Warning: child %d has dfa state = %d for abort tsc = %llu\n", __tmp_coreid, childState, abort_tsc);
                            for_each_thread_in_core(__tmp_coreid, threadid) {
                                db_fprintf(stderr, "Core id = %d, thread id = %d\n", __tmp_coreid, threadid);
                                snapshot.threadSnapshots[threadid] = s_emptyThread;
                            }
                        } else {
                            db_fprintf(stderr, "child %d has OK dfa state\n", __tmp_coreid);
                        }
                    }
                }
                if (g_do_debugging) {
                    std::cerr << "Snapshot after reset ..." << snapshot;
                }
            };
    };

    class Core : public Processor {
        private:
            int m_lastTpsSampleID;
        public:
            Core(int id, Processor *package=NULL) : Processor("Core", id), m_lastTpsSampleID(-1) { 
                if (package) {
                    package->add_child(this);
                }
            };

        private:
            void update_prev_snapshot_i(IdleSnapshot& snapshot, const PWCollector_sample_t *sample)
            {
                snapshot.tps = sample;
                snapshot.req_state = C9;
                snapshot.m_minTSC = snapshot.m_maxTSC = 0x0;
                for (std::map <int, ThreadSnapshot>::const_iterator citer = snapshot.threadSnapshots.begin(); citer != snapshot.threadSnapshots.end(); ++citer) {
                    snapshot.req_state = std::min(snapshot.req_state, citer->second.get_req_state());
                    pw_u64_t __thread_tsc = citer->second.get_tsc();
                    if (__thread_tsc) {
                        snapshot.m_minTSC = std::min(snapshot.m_minTSC, __thread_tsc);
                    }
                    snapshot.m_maxTSC = std::max(snapshot.m_maxTSC, __thread_tsc);
                }
            };
            void create_rewind_sample_and_update_prev_snapshot_i(const PWCollector_sample_t *__sample)
            {
                if (unlikely(m_idleSnapshots.size() < 2)) {
                    return;
                }
                /*
                 * Basic sanity check
                 */
                if (m_samples[C_STATE].empty() == false && m_samples[C_STATE].back().tsc == __sample->tsc) {
                    db_fprintf(stderr, "Warning: possible error in rewind mechanism detected at rewind tsc == %llu\n", __sample->tsc);
                    db_abort("Warning: possible error in rewind mechanism detected at rewind tsc == %llu\n", __sample->tsc);
                    return;
                }
                const IdleSnapshot& curr = m_idleSnapshots.back();
                IdleSnapshot& prev = *(m_idleSnapshots.end() - 2);
                PWCollector_sample_t tps_sample;
                const PWCollector_sample_t *wu_cause = curr.wu_cause;
                /*
                 * TSC (tps) == TSC(__sample)
                 * Req-state = ?; act_state = C1
                 * Wu-cause from current snapshot;
                 */
                int cpuidx = __sample->cpuidx;
                c_state_t req_state = C9, act_state = APERF;
                pw_u64_t delta_tsc = __sample->tsc - prev.tps->tsc, c0_res = 0, cx_res = 0;
                pw_u64_t mperf = 0;
                for (std::map<int,ThreadSnapshot>::iterator iter = prev.threadSnapshots.begin(); iter != prev.threadSnapshots.end(); ++iter) {
                    c_state_t __state = iter->second.req_state;
                    if (iter->first == cpuidx) {
                        __state = (c_state_t)GET_C_STATE_GIVEN_TPS_HINT(__sample->c_sample.prev_state);
                        mperf = iter->second.get_mperf();
                        iter->second = ThreadSnapshot(__sample);
                    }
                    req_state = std::min(req_state, __state);
                }
                {
                    update_prev_snapshot_i(prev, __sample);
                }
                assert(mperf);
                if (unlikely(prev.m_wasAbort)) {
                    c0_res = delta_tsc;
                } else {
                    c0_res = RES_COUNT(__sample->c_sample, MPERF) - mperf;
                }
                if (c0_res > delta_tsc) {
                    c0_res = delta_tsc;
                }
                cx_res = delta_tsc - c0_res; // C1 == Delta-TSC - Delta-MPERF
                create_c_state_sample_i(tps_sample, __sample->tsc, req_state, act_state, c0_res, cx_res, wu_cause);
                if (unlikely(cx_res == 0x0 && tps_sample.c_sample.break_type != PW_BREAK_TYPE_B)) {
                    tps_sample.c_sample.break_type = PW_BREAK_TYPE_A;
                }
                if (unlikely(prev.m_wasAbort)) {
                    assert(tps_sample.c_sample.break_type == PW_BREAK_TYPE_A); // sanity!
                }
                if (g_do_debugging) {
                    std::cerr << "OK: created new rewind sample: " << tps_sample;
                }
                m_samples[C_STATE].push_back(tps_sample);
                /*
                 * We'll need to notify our children that we entered idle.
                 */
                m_parentIdleSnapshotIdx = (int)m_idleSnapshots.size() - 1;
            };
#if 0
            void create_rewind_sample_i(const PWCollector_sample_t *__sample)
            {
                if (unlikely(m_idleSnapshots.size() < 2)) {
                    return;
                }
                const IdleSnapshot& curr = m_idleSnapshots.back();
                const IdleSnapshot& prev = *(m_idleSnapshots.end() - 2);
                PWCollector_sample_t tps_sample;
                const PWCollector_sample_t *wu_cause = curr.wu_cause;
                /*
                 * TSC (tps) == TSC(__sample)
                 * Req-state = ?; act_state = C1
                 * Wu-cause from current snapshot;
                 */
                int cpuidx = __sample->cpuidx;
                c_state_t req_state = C9, act_state = APERF;
                pw_u64_t delta_tsc = __sample->tsc - prev.tps->tsc, c0_res = 0, cx_res = 0;
                pw_u64_t mperf = 0;
                for (std::map<int,ThreadSnapshot>::const_iterator citer = prev.threadSnapshots.begin(); citer != prev.threadSnapshots.end(); ++citer) {
                    c_state_t __state = citer->second.req_state;
                    if (citer->first == cpuidx) {
                        __state = (c_state_t)GET_C_STATE_GIVEN_TPS_HINT(__sample->c_sample.prev_state);
                        mperf = citer->second.get_mperf();
                    }
                    req_state = std::min(req_state, __state);
                }
                assert(mperf);
                if (unlikely(prev.m_wasAbort)) {
                    c0_res = delta_tsc;
                } else {
                    c0_res = RES_COUNT(__sample->c_sample, MPERF) - mperf;
                }
                if (c0_res > delta_tsc) {
                    c0_res = delta_tsc;
                }
                cx_res = delta_tsc - c0_res; // C1 == Delta-TSC - Delta-MPERF
                create_c_state_sample_i(tps_sample, __sample->tsc, req_state, act_state, c0_res, cx_res, wu_cause);
                if (unlikely(cx_res == 0x0 && tps_sample.c_sample.break_type != PW_BREAK_TYPE_B)) {
                    tps_sample.c_sample.break_type = PW_BREAK_TYPE_A;
                }
                if (unlikely(prev.m_wasAbort)) {
                    assert(tps_sample.c_sample.break_type == PW_BREAK_TYPE_A); // sanity!
                }
                /*
                if (unlikely(prev.m_wasAbort)) {
                    tps_sample.c_sample.break_type = PW_BREAK_TYPE_A;
                    db_fprintf(stderr, "Changed: %llu to ABORT! C0 res = %llu\n", tps_sample.tsc, RES_COUNT(tps_sample.c_sample, MPERF));
                }
                if (unlikely(cx_res == 0x1)) {
                    if (g_do_debugging) {
                        std::cerr << "POSSIBLE ABORT detected in rewind sample: " << tps_sample;
                    }
                    tps_sample.c_sample.break_type = PW_BREAK_TYPE_A;
                }
                */
                if (g_do_debugging) {
                    std::cerr << "OK: created new rewind sample: " << tps_sample;
                }
                m_samples[C_STATE].push_back(tps_sample);
                /*
                 * We'll need to notify our children that we entered idle.
                 */
                m_parentIdleSnapshotIdx = m_idleSnapshots.size() - 1;
            };
#endif
            /*
             * INTERNAL API:
             * Core-specific callback for 'idle'-related samples (TPS, IRQ etc.)
             */
            int handle_idle_sample_i(const PWCollector_sample_t *sample) {
                /*
                 * Special case for CLTP: handle the case where our parent signals
                 * a package C-state entry, but our idle snapshot is either incomplete
                 * or is completely invalid because that indicates a core C-state abort.
                 */
                if (m_parentAbortTSC) {
                    IdleSnapshot& idleSnapshot = m_idleSnapshots.back();
                    db_fprintf(stderr, "%s[%d]: parent abort tsc = %llu\n", m_typeString.c_str(), m_id, m_parentAbortTSC);
                    if (g_do_debugging) {
                        std::cerr << "Current snapshot: " << idleSnapshot;
                    }
                    if (m_parentAbortTSC > idleSnapshot.tps->tsc) {
                        db_fprintf(stderr, "\t[%d]: Abort > current snapshot TSC; NOTHING TO DO!\n", m_id);
                    } else if (m_parentAbortTSC == idleSnapshot.tps->tsc) {
                        db_fprintf(stderr, "\t[%d]: Abort == current snapshot TSC! NEED TO RESET CURRENT SNAPSHOT\n", m_id);
                        idleSnapshot.m_wasAbort = true;
                        db_fprintf(stderr, "Confirmed = %s\n", GET_BOOL_STRING(IS_CONFIRMED_IDLE(m_idleSnapshots.back())));
                    } else if (m_idleSnapshots.size() > 1) {
                        std::vector<IdleSnapshot>::iterator iter = m_idleSnapshots.end() - 2;
                        if (g_do_debugging) {
                            std::cerr << "previous snapshot: " << *iter;
                        }
                        if (m_parentAbortTSC != iter->tps->tsc) {
                            db_fprintf(stderr, "\t[%d]: Abort != previous snapshot TSC; NOTHING TO DO!\n", m_id);
                        } else {
                            db_fprintf(stderr, "\t[%d]: Abort == previous snapshot TSC! NEED TO DISCARD PREVIOUS SNAPSHOT\n", m_id);
                            iter->m_wasAbort = true;
                        }
                    }
                    m_parentAbortTSC = 0x0;
                }
                if (m_parentIdleTSC) {
                    assert(m_parentIdleSample->tsc == m_parentIdleTSC);
                    if (likely(m_idleSnapshots.empty() == false)) {
                        //pw_u64_t tsc = 0x0;
                        pw_u64_t pkg_tsc = m_parentIdleTSC;
                        IdleSnapshot& snapshot = m_idleSnapshots.back();
                        /*
                         * We do NOT rewind the snapshot if the current core idle TSC ...
                         * (i) ...started AFTER pkg idle TSC OR
                         * (ii) ... ended AT/BEFORE pkg idle TSC
                         *
                         * For (i) to be true: pkg TSC < snapshot.m_minTSC
                         * For (ii) to be true: pkg TSC >= snapshot.m_maxTSC
                         *
                         * Note these equations are only valid if all threads on this core have entered idle,
                         * which is why we also check the 'req_state' field (this will be MPERF until
                         * all threads have entered idle).
                         */
                        if (pkg_tsc < snapshot.m_minTSC || (snapshot.req_state > MPERF && pkg_tsc >= snapshot.m_maxTSC)) {
                            // No need to rewind
                            if (g_do_debugging) {
                                std::cerr << "[" << m_id << "]: NO-REWIND for tsc = " << pkg_tsc << " in current snapshot = " << snapshot;
                            }
                        } else {
                            if (g_do_debugging) {
                                std::cerr << "[" << m_id << "]: REWIND REQUIRED for tsc = " << pkg_tsc << " in current snapshot = " << snapshot;
                                std::cerr << "Current DFA state = " << m_currDfaState << "\n";
                            }
                            /*
                             * Basic algo:
                             * REQUIRED: The LARGEST TSC of any TPS on the current core that is still <= pkg_tsc.
                             * i.e. the mwait on the current core that is closest to 'pkg_tsc' while still being less than
                             * it.
                             * SOLUTION:
                             * 1. Package belongs to current core?
                             * a. YES => Use package sample
                             * b. NO => Look at current snapshot: is there a sample 's' s.t. TSC(s) < pkg_tsc 
                             * (there can be at most one such sample because otherwise the snapshot is VALID 
                             * and doesn't need to be rewound).
                             * i. YES => Use 's'
                             * ii. NO => Look at the only thread present in the current snapshot (there can be
                             * only one thread otherwise condition 1.b.i above would be true). Get the PREVIOUS
                             * idle snapshot/sample for this thread. Use that sample.
                             *
                             * Once the sample has been found, rewind the current snapshot and create a new
                             * C1 C-state sample with TSC == TSC("sample")
                             */
                            {
                                Processor *thread = get_thread_given_id_i(m_parentIdleSample->cpuidx);
                                IdleSnapshot __snapshot;
                                const PWCollector_sample_t *__sample = NULL;
                                //pw_u64_t __tsc = 0x0, __c0_res = 0x0, __cx_res = 0x0;
                                //pw_u32_t __req_state = 0x0, __act_state = 0x0;
                                if (thread) {
                                    /*
                                     * Case 1.a
                                     */
                                    __sample = m_parentIdleSample;
                                }
                                {
                                    /*
                                     * Case 1.b + current snapshot rewind
                                     */
                                    //const PWCollector_sample_t *__tmp_sample = NULL;
                                    int __child_id = -1, __num_children = 0, __num_reset = 0;
                                    pw_u8_t __dfa_state = DFA_C_ZERO;
                                    for (std::map<int, ThreadSnapshot>::iterator iter = snapshot.threadSnapshots.begin(); iter != snapshot.threadSnapshots.end(); ++iter) {
                                        const ThreadSnapshot& __snapshot = iter->second;
                                        pw_u64_t __tmp = __snapshot.get_tsc();
                                        if (__tmp && __tmp <= pkg_tsc) {
                                            if (!__sample) {
                                                /*
                                                 * Case 1.b.i
                                                 */
                                                __sample = __snapshot.get_sample();
                                            }
                                            /*
                                             * Rewind.
                                             */
                                            iter->second = s_emptyThread;
                                            snapshot.req_state = MPERF; // Reflects the fact that we haven't seen all idles on this core yet.
                                            /*
                                             * Also need to recalc the 'm_currDfaState'!!!
                                             */
#if 0
                                            if (m_currDfaState > DFA_C_ZERO) {
                                                /*
                                                 * We need to take into account the fact that we've just 'reset' the 
                                                 * effects of a TPS sample. If current state is 'DFA_C_X' then change to
                                                 * 'DFA_C_HALF'; if it is 'DFA_C_HALF' then change to 'DFA_C_ZERO'.
                                                 */
                                                m_currDfaState = (DFA_state)(((int)m_currDfaState) - 1);
                                                db_fprintf(stderr, "%s[%d]: reset state to %u\n", m_typeString.c_str(), m_id, m_currDfaState);
                                            }
#else
                                            {
                                                ++__num_reset;
                                            }
#endif
                                        } else if (__tmp) {
                                            __child_id = iter->first;
                                            ++__num_children;
                                            ++__dfa_state;
                                        }
                                    }
                                    if (!__sample) {
                                        /*
                                         * Case 1.b.ii
                                         */
                                        // assert(__num_children == 1);
                                        Processor *__child = NULL;
                                        FOR_EACH_CHILD_OF(this, __child) {
                                            if (__child->get_id() == __child_id) {
                                                __sample = __child->get_rewound_sample(pkg_tsc);
                                            }
                                        }
                                    }
                                    m_currDfaState = (DFA_state)__dfa_state;
                                }
                                if (unlikely(!__sample)) {
                                    // This should only happen if we dropped some samples.
                                    // assert(pwr::WuData::instance()->getSystemInfo().m_droppedSamples > 0);
                                } else {
                                    assert(__sample);
                                    if (g_do_debugging) {
                                        std::cerr << "REWIND SAMPLE: " << *__sample;
                                    }
                                    /*
                                     * OK, retrieved rewind sample. Now use this sample (and the previous idle snapshot) to
                                     * create a new C1 c-state sample.
                                     */
                                    // create_rewind_sample_i(__sample);
                                    create_rewind_sample_and_update_prev_snapshot_i(__sample);
                                }
                            }
                        }
                    }
                    m_parentIdleTSC = 0; m_parentIdleSample = NULL;
                }
                /*
                 * We've received a sample that could potentially alter our DFA state.
                 */
                update_non_leaf_dfa_state_i(sample);
                return PW_SUCCESS;
            };
            /*
             * INTERNAL API:
             * Core-specific callback for P-state samples.
             */
#define IS_CRITICAL_SAMPLE(sample) ( GET_REQ_FREQ_FROM_COMBINED(sample) < CLTP_CRITICAL_FREQUENCY() && GET_ACT_FREQ_FROM_COMBINED(sample) == CLTP_CRITICAL_FREQUENCY() )
#define IS_NODE_IN_CRITICAL_FREQUENCY() ( m_currReqFreq < CLTP_CRITICAL_FREQUENCY() && m_currActFreq == CLTP_CRITICAL_FREQUENCY() )
            int handle_freq_sample_i(const PWCollector_sample_t *sample) {
                update_non_leaf_freq_state_i(sample);
                /*
                 * CLTP ONLY! Let our parent (i.e. the "package") handle all
                 * P-state samples.
                 */
                if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                    db_fprintf(stderr, "%s[%d]: req freq = %u, act freq = %u\n", m_typeString.c_str(), m_id, m_currReqFreq, m_currActFreq);
                    if (unlikely(m_currReqFreq > m_currActFreq)) {
                        db_fprintf(stderr, "Warning: at TSC = %llu, req freq = %u is MORE than act freq = %u\n", sample->tsc, m_currReqFreq, m_currActFreq);
                    }
                    if (unlikely(IS_NODE_IN_CRITICAL_FREQUENCY())) {
                        db_fprintf(stderr, "Debug: at TSC = %llu, node is in CRITICAL FREQUENCY! Req = %u\n", sample->tsc, m_currReqFreq);
                        m_primeFreq = m_currReqFreq;
                    } else {
                        m_primeFreq = m_currActFreq;
                    }
                    db_fprintf(stderr, "Prime = %u\n", m_primeFreq);
                    return PW_SUCCESS;
                }
                m_samples[P_STATE].push_back(*sample);
                PWCollector_sample_t& tpf_sample = m_samples[P_STATE].back();
                /*
                 * Ensure all TPF samples have the core ID in the 'cpuidx' field.
                 */
                tpf_sample.cpuidx = m_id;
                return PW_SUCCESS;
            };
            /*
             * INTERNAL API:
             * Core-specific callback for hotplug state samples.
             */
            int handle_cpuhotplug_sample_i(const PWCollector_sample_t *sample) {
                m_samples[CPUHOTPLUG_SAMPLE].push_back(*sample);
                PWCollector_sample_t& cpuhotplug_sample = m_samples[CPUHOTPLUG_SAMPLE].back();
                /*
                 * Ensure all TPF samples have the core ID in the 'cpuidx' field.
                 */
                cpuhotplug_sample.cpuidx = m_id;
                return PW_SUCCESS;
            };
            /*
             * INTERNAL API:
             * Function to add a mask to individual samples to identify them as being
             * core-specific.
             * @cpuid: the field to which a mask is added.
             */
            void add_topology_mask_to_cpuid_i(pw_u32_t& cpuid) {
                cpuid |= CORE_CPUIDX_MASK;
            };
            /*
             * INTERNAL API:
             * Core-specific check to distinguish between valid and invalid idles. An example of an
             * 'invalid' idle is when the core requests a deep sleep state, but the Cx MSRs didn't
             * count (indicating the H/W never granted the OS request).
             * @req_state: the C-state requested by the core when it last entered idle.
             * @act_state: the C-state actually granted by H/W.
             *
             * @returns: 'true' ==> the previous idle was valid; 'false' otherwise
             */
            bool is_valid_idle_i(c_state_t req_state, c_state_t act_state) {
                if (m_wasSaltwell || m_wasAutoDemoteEnabled) {
                    return IS_VALID_AUTO_DEMOTE_ENABLED_IDLE(req_state, act_state);
                }
                return IS_VALID_AUTO_DEMOTE_DISABLED_IDLE(req_state, act_state);
            };
            void update_dfa_c_states_i(const PWCollector_sample_t *sample)
            {
                if (m_wasSaltwell == false) {
                    calculate_dfa_c_states_i();
                    return;
                }
                DFA_state oldState = m_currDfaState;
                if (sample->sample_type == C_STATE) {  // TPS
                    switch (m_currDfaState) {
                        case DFA_C_ZERO:
                            m_lastTpsSampleID = sample->cpuidx;
                            /*
                             * If we have multiple threads then move to C_HALF. If we have only one thread
                             * then move to C_X.
                             */
                            m_currDfaState = WAS_SYSTEM_HYPER_THREADED() ? DFA_C_HALF : DFA_C_X;
                            break;
                        case DFA_C_HALF:
                            if (m_lastTpsSampleID == sample->cpuidx) { // STPS
                                // NOP
                            } else { // OTPS
                                m_lastTpsSampleID = -1;
                                m_currDfaState = DFA_C_X;
                            }
                            break;
                        case DFA_C_X:
                            /*
                             * If we have multiple threads then move to C_HALF. If we have only one thread
                             * then move back to C_X.
                             */
                            m_lastTpsSampleID = sample->cpuidx;
                            m_currDfaState = WAS_SYSTEM_HYPER_THREADED() ? DFA_C_HALF : DFA_C_X;
                            break;
                    }
                } else { // NTPS
                    switch (m_currDfaState) {
                        case DFA_C_ZERO:
                            // NOP
                            break;
                        case DFA_C_HALF:
                            if (m_lastTpsSampleID == sample->cpuidx) { // SNTPS
                                m_lastTpsSampleID = -1;
                                m_currDfaState = DFA_C_ZERO;
                            } else { // ONTPS
                                // NOP
                            }
                            break;
                        case DFA_C_X:
                            m_lastTpsSampleID = -1;
                            m_currDfaState = DFA_C_ZERO;
                            break;
                    }
                }
                db_fprintf(stderr, "Core[%d]: oldState = %d, sample type = %d, newState = %d\n", m_id, oldState, sample->sample_type, m_currDfaState);
            };
    };

    class Thread : public Processor {
        public:
            Thread(int id, Processor *core=NULL) : Processor("Thread", id) {
                if (core) {
                    core->add_child(this);
                }
            };
            ~Thread() {
            };
        private:
            void take_thread_snapshot_i(const PWCollector_sample_t *sample) {
                assert (sample->sample_type == C_STATE);
                m_prevPrevSnapshot = m_prevSnapshot;
                m_prevSnapshot = m_currSnapshot;
                m_currSnapshot = ThreadSnapshot(sample);
                return;
            };
            void create_idle_snapshot_i(IdleSnapshot& idleSnapshot, int curr_tps_cpuid) {
                /*
                 * We require ALL threads on a core/package to enter idle before we consider the core/package
                 * to be in idle. Note that his applies only to {NHM/WMR/SNB} C1 idles and to Saltwell {MFD/LEX/CLTP} 
                 * Package Cx idles.
                 */
                ThreadSnapshot& threadSnapshot = idleSnapshot.threadSnapshots[m_id];
                if (m_id == curr_tps_cpuid) {
                    threadSnapshot = m_currSnapshot;
                    idleSnapshot.m_minTSC = idleSnapshot.m_maxTSC = idleSnapshot.m_currMinTSC = m_currSnapshot.get_tsc();
                } else {
                    threadSnapshot = s_emptyThread;
                }
                idleSnapshot.req_state = std::min(idleSnapshot.req_state, threadSnapshot.get_req_state());
                return;
            };

            int handle_idle_sample_i(const PWCollector_sample_t *sample) {
                bool is_tps = sample->sample_type == C_STATE;
                /*
                 * Next DFA state for leaf nodes governed entirely by the 'sample'
                 */
                if (is_tps) {
                    copy_msr_set_i(&RES_COUNT(sample->c_sample, MPERF));
                    m_currReqState = (c_state_t)GET_C_STATE_GIVEN_TPS_HINT(sample->c_sample.prev_state);
                    m_currDfaState = DFA_C_X;
                    take_thread_snapshot_i(sample);
                } else {
                    m_currReqState = C0;
                    m_currDfaState = DFA_C_ZERO;
                    db_fprintf(stderr, "[%d]: reset leaf state at tsc = %llu\n", m_id, sample->tsc);
                }
                return PW_SUCCESS;
            };

            void get_rewound_thread_snapshot(ThreadSnapshot& snapshot, const pw_u64_t tsc) {
                if (m_currSnapshot.get_tsc() && m_currSnapshot.get_tsc() <= tsc) {
                    snapshot = m_currSnapshot;
                } else if (m_prevSnapshot.get_tsc() && m_prevSnapshot.get_tsc() <= tsc) {
                    snapshot = m_prevSnapshot;
                } else {
                    snapshot = m_prevPrevSnapshot;
                }
            };

            const PWCollector_sample_t *get_rewound_sample(const pw_u64_t tsc) {
                // At the very least we should have a 'current' snapshot!
                pw_u64_t __tsc = m_currSnapshot.get_tsc();
                assert(__tsc);
                if (__tsc <= tsc) {
                    return m_currSnapshot.get_sample();
                } else {
                    // We only need to check the 'previous' snapshot
                    __tsc = m_prevSnapshot.get_tsc();
                    if (__tsc <= tsc) {
                        return m_prevSnapshot.get_sample();
                    }
                }
                // assert(false);
                return NULL;
            };

            void calculate_dfa_c_states_i() {
                // NOP
            };
    };

    /*
     * Parser class -- reads wuwatch data, parses it and converts v3 pwr samples to v2 format.
     */
    class WuParser {
        private:
            /*
             * The combined output file name. Under new
             * scheme, both driver output and sys params
             * info gets dumped into a single file (with
             * a ".ww<X>" extension).
             */
            std::string m_combined_input_file_name;
            /*
             * Where should we store intermediate output?
             */
            std::string m_output_file_name;
            /*
             * A list of valid frequencies. We will eventually
             * get this either from the driver, or from
             * the "sys" file system. For now, default to MFLD
             * values.
             * **********************************************
             * WARNING: LAST ENTRY IN ARRAY MUST BE ZERO!!!
             * **********************************************
             */
            std::vector <u32> m_availableFrequenciesKHz;
            /*
             * # cpus on TARGET machine
             * i.e. on machine where experiment
             * was conducted.
             */
            int PW_max_num_cpus;
            /*
             * List of PWCollector_sample_t instances
             * output by the DD.
             */
            sample_vec_t m_origSamples;
            /*
             * A list of post-processed samples, sorted in TSC order.
             */
            sample_list_t m_output_samples;
            /*
             * Flag to indicate whether
             * we should dump backtrace
             * information.
             */
            bool m_do_dump_backtraces;
            /*
             * Should we dump original samples i.e. samples as returned by
             * the driver (without any post-processing other than basic
             * sorting wrt tsc)?
             */
            bool m_do_dump_orig_samples;
            /*
             * Should we dump sample statistics? Currently we dump only
             * the following: total # samples collected, # dropped samples,
             * # package samples, # package overcounts.
             */
            bool m_do_dump_sample_stats; 
            /*
             * Driver <major, minor, other> version number.
             */
            pw_s32_t m_driverMajor;
            pw_s32_t m_driverMinor;
            pw_s32_t m_driverOther;
            /*
             * The bus clock frequency, in KHz.
             */
            pw_u32_t m_busClockFreqKHz;
            /*
             * The bus clock frequency, in MHz;
             */
            float m_busClockFreqMHz;
            /*
             * The bitmask to use to extract actual frequency values from 'IA32_PERF_STATUS'.
             */
            pw_u16_t m_perfStatusMask;
            /*
             * Should we convert S/D residency 'usecs' to TSC ticks?
             * Used ONLY when importing wuwatch data into AXE.
             */
            bool m_do_convert_s_d_res_to_ticks;
            /*
             * Were we called from 'AXE'?
             */
            bool m_is_from_axe;
            /*
             * We need to keep track of the total collection time. This is merely
             * the TSC of the last C-state sample - TSC of the first core mwait.
             * We need a bit of extra logic because the collection time is really
             * the MAX of the collection times of the individual cores.
             */
            u64 m_total_collection_time;
            /*
             * A list of per-thread "MSR sets". Required for versions >= 3.1.0
             */
            msr_set_map_t m_per_thread_msr_sets;
            /*
             * A list of per-thread clock_gettime / TSC ratios.
             * Required for versions >= 3.1.0
             */
            std::map <int, double> m_per_thread_clock_ratios;
            /*
             * Wakelock constant pools. We have separate ones for userspace and kernel space.
             */
            std::map <pw_u32_t, std::string> m_kernelWakelockConstantPool;
            std::map <pw_u32_t, std::string> m_userWakelockConstantPool;
            std::map <pw_u32_t, std::vector<PWCollector_sample_t> > m_incompleteWakelockMessages;
            /*
             * The number of 'W_STATE' samples created by parsing the "/proc/wakelocks" file.
             * Note that any such sample will ALWAYS have a wakelock type of: 'PW_WAKE_LOCK_INITIAL'
             */
            pw_s32_t m_maxProcWakelocksConstantPoolIndex;
            /*
             * The number of threads, cores, modules and packages on the target system.
             * Used to construct the CPU 'topology'.
             */
            int m_threadCount, m_coreCount, m_moduleCount, m_packageCount;
            /*
             * The list of 'Thread', 'Core' and 'Package' Processor nodes that
             * comprise the topology of the target machine.
             */
            // std::vector <Processor *> m_packages, m_modules, m_cores, m_threads;
            std::map <int, Processor *> m_packages, m_modules, m_cores, m_threads;
            /*
             * The set of thread-level, core-level and package-level MSRs supported by
             * the target platform. Each vector is always MAX_MSR_ADDRESSES entries long.
             * A '1' in entry 'x' denotes the target platform supports MSR Cx.
             */
            std::vector <int> m_thread_level_supported_msrs, m_core_level_supported_msrs, m_module_level_supported_msrs, m_package_level_supported_msrs;
            /*
             * Repository for system configuration info.
             */
            SystemInfo& sysInfo;

            /*
             * Friends.
             */
            friend class WuData;

            pw_u64_t prev_s_res[6];
            pw_u64_t prev_d_res[MAX_LSS_NUM_IN_SC][4];

        private:
            int do_read_i();
            int do_parse_i();
            int do_finalize_and_collate_i();
            int do_parse_messages_i(std::vector<char>& data_buffer, size_t count, std::vector <PWCollector_sample_t>& samples, int& buffer_offset, u64& num_samples);
            int do_convert_msg_to_sample_i(PWCollector_msg_t *msg, std::vector <PWCollector_sample_t>& samples);

            // template <typename InputIterator> int do_process_samples_i(InputIterator from, InputIterator to);

            int get_sys_params_i(FILE *, u64&);

            std::string get_required_token_i(std::string , int , int , const char *);
            void dump_pwcollector_sample_i(const PWCollector_sample_t& pwc_sample);

#if DO_TPS_EPOCH_COUNTER
            void do_sort_sched_list_i(int core);
            void do_merge_sched_list_i(int core);
#endif // DO_TPS_EPOCH_COUNTER

            /* ***********************************************
             * CPU topology and sample parsing functions.
             * ***********************************************
             */
            void build_topology_i();
            int replay_trace_i(const std::vector <PWCollector_sample_t>& trace);
            void set_thread_level_msrs_i(const std::vector <int>& thread_msrs);
            void set_core_level_msrs_i(const std::vector <int>& core_msrs);
            void set_package_level_msrs_i(const std::vector <int>& package_msrs);
            void finalize_samples_i(std::map<sample_type_t, sample_list_t>& samples);
            void post_process_samples_i(sample_list_t& samples, sample_type_t type);
            void traverse_i();
            void set_msrs_i(const std::vector<int>& msrs, std::map <int, Processor *>& which);


        public:

            WuParser(SystemInfo&, bool, bool, bool, bool, bool);
            ~WuParser();

            int do_work();
            int do_work(sample_list_t&);

            void set_wuwatch_output_file_name(const std::string&);
    };
} // pwr


/* *****************************************
 * Some useful function declarations.
 * *****************************************
 */
void operator<<(std::ostream& os, const int_pair_t& p);
bool operator==(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2);
bool operator!=(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2);
bool operator<(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2);

/* *****************************************
 * Variable declarations.
 * *****************************************
 */
/*
 * For HT-testing. These correspond to the various
 * "sample_type_t" enum values. We have a long and 
 * a short version of each name.
 */
static const char *s_long_sample_names[] = {"FREE_SAMPLE", "TPS", "TPF", "K_CALL", "M_MAP", "I_MAP", "P_MAP", "S_RES", "S_ST", "D_RES", "D_ST", "TIM", "IRQ", "WRQ", "SCD", "IPI", "TPE", "W_ST", "D_MAP", "MSR", "U_ST", "PSX", "CPE", "PKG_MAP", "CPUHOTPLUG", "SAMPLE_END"};

static unsigned int max_lss_num_in_nc = 0;
static unsigned int max_lss_num_in_sc = 0;

/* *****************************************
 * Function definitions.
 * *****************************************
 */
bool CheckEnv(const char *name)
{
#if defined(_WIN32)
//    char value[128];
    char *pValue = NULL;
    size_t len;
    errno_t err;
       err = _dupenv_s(&pValue, &len, name);
    if (err) { 
        return false;
    } else {
//        if (pValue == NULL || len == 0) {
//            return false;
//        }
//        err = memcpy_s(value, 128, pValue, 127);
        bool ret = (pValue && (!strcmp(pValue, "y") || !strcmp(pValue, "Y"))) ? true : false;
        if (pValue != NULL) {
            free(pValue);
        }
//        if (err) { 
//            return false;
//        }
//        return (value && (!strcmp(value, "y") || !strcmp(value, "Y"))) ? true : false;
        return ret;
    }
#else
    const char *value = getenv(name);
    if (value == NULL) {
        return false;
    } else {
        return (value && (!strcmp(value, "y") || !strcmp(value, "Y"))) ? true : false;
    }
#endif
}

/*
 * The parser constructor.
 */
pwr::WuParser::WuParser(SystemInfo& info, bool should_calc_c1, bool should_dump_orig, bool should_dump_stats, bool should_convert_usecs_to_tsc, bool is_from_axe) : sysInfo(info),
    PW_max_num_cpus(-1), 
    m_do_dump_backtraces(false), m_do_convert_s_d_res_to_ticks(should_convert_usecs_to_tsc), m_is_from_axe(is_from_axe), 
    m_do_dump_orig_samples(should_dump_orig), m_do_dump_sample_stats(should_dump_stats),
    m_driverMajor(0), m_driverMinor(0), m_driverOther(0),
    m_busClockFreqKHz(0), m_busClockFreqMHz(0.0), m_perfStatusMask(0), m_total_collection_time(0),
    m_combined_input_file_name(""), m_output_file_name("./parser_output.txt"),
    m_maxProcWakelocksConstantPoolIndex(-1), m_threadCount(0), m_coreCount(0), m_moduleCount(0), m_packageCount(0) {};

/*
 * The parser destructor.
 */
pwr::WuParser::~WuParser()
{
    traverse_i();

    Processor *proc;
    FOR_EACH_PROC(proc, m_packages) {
        delete (Package *)proc;
    }
};

/*
 * Instance functions for helper
 * structs declared earlier.
 */
pwr::SystemInfo::SystemInfo():m_microPatchVer(0) {};
pwr::SystemInfo::~SystemInfo() {};

pwr::WuData *pwr::WuData::s_data = NULL;


/**********************************************
  WUDUMP FUNCTIONS
 **********************************************/

/*
 * INTERNAL API:
 * Helper function: tokenize 'line', using delim set
 * 'd', and return the 'tok_num'th token.
 *
 * @line: the string to tokenize
 * @tok_num: the token number to return
 * @max_num_toks: the max # of tokens to expect in @line
 * @d: the (possibly empty) delim set.
 *
 * @returns: the 'tok_num'th token in 'line'
 */
std::string pwr::WuParser::get_required_token_i(std::string line, int tok_num, int max_num_toks, const char *d = NULL)
{
    const char *delims = d ? d : " ";
    str_vec_t toks = Tokenizer(line, delims).get_all_tokens();

    db_assert(toks.size() == (size_t)max_num_toks, "ERROR: invalid # toks = %lu (expected %lu)\n", TO_UL(toks.size()), TO_UL(max_num_toks));

    return toks[tok_num - 1];
};

/*
 * INTERNAL API:
 * Read collection/system params from the 'sys_params_found.txt'
 * file -- adapted from 'wudump' code.
 *
 * @in_fp: the file to read.
 * @sys_params_off: the offset (within the wuwatch ".ww1" output file) from
 * which to begin reading system configuration information.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::get_sys_params_i(FILE *in_fp, u64& sys_params_off)
{

#define DOES_LINE_CONTAIN(line, str) ( (line).find(str) != std::string::npos )

    str_deq_t lines;
    /*
     * Make sure the 'init()'
     * function has been called!
     */
    db_assert(m_combined_input_file_name.size() > 0, "ERROR: init not called before get_sys_params()!\n");
    /*
     * Starting offset of system param information is
     * encoded in first 8 bytes of the combined output file.
     * Read that here.
     */
    sys_params_off = 0;
    if (fread(&sys_params_off, sizeof(sys_params_off), 1, in_fp) != 1) {
        db_perror("fread error in get_sys_params_i()");
        return -PW_ERROR;
    }
    db_fprintf(stderr, "SYS PARAMS OFFSET = %llu\n", TO_ULL(sys_params_off));
    /*
     * OK, seek to the offset specified.
     */
    if (fseek(in_fp, (long)sys_params_off, SEEK_SET)) {
        db_perror("fseek error");
        return -PW_ERROR;
    }
    /*
     * Next line MUST be "--SYS_PARAMS_BEGIN---"!!!
     */
    std::string line;
    if (LineReader::getline(in_fp, line) < 0) {
        db_fprintf(stderr, "my_getline error");
        return -PW_ERROR;
    }
    db_fprintf(stderr, "NEXT LINE IS: %s\n", line.c_str());
    assert(!strcmp(line.c_str(), "---SYS_PARAMS_BEGIN---"));
    /*
     * OK, we're ready to begin. Start by reading in
     * all the remaining lines in the input file. Then parse
     * them to extract system configuration information.
     */
    LineReader::get_all_lines(in_fp, lines);

    /*
     * Users expect to see a copy of the system config in its entirety.
     * Make a copy here.
     */
    sysInfo.m_lines = lines;

    while(!lines.empty()) {
        str_t line = lines.front(); lines.pop_front();
        db_fprintf(stderr, "%s\n", line.c_str());

        if (line.find("Version") != std::string::npos) {
            /*
             * Found Driver/Wuwatch version info.
             */
            str_vec_t toks = Tokenizer(line, " ").get_all_tokens();
            db_assert(toks.size() == (size_t)4, "ERROR: invalid # tokens = %lu\n", TO_UL(toks.size()));
            str_t ver_str = toks[3];
            if (toks[0] == "Driver") {
                str_vec_t driverToks = Tokenizer(ver_str, ".").get_all_tokens();
                assert(driverToks.size() == 3);
                m_driverMajor = atoi(driverToks[0].c_str());
                m_driverMinor = atoi(driverToks[1].c_str());
                m_driverOther = atoi(driverToks[2].c_str());
                sysInfo.m_driverMajor = m_driverMajor;
                sysInfo.m_driverMinor = m_driverMinor;
                sysInfo.m_driverOther = m_driverOther;
                sysInfo.m_driverVersion = ver_str;
            } else if (toks[0] == "Wuwatch") {
                sysInfo.m_wuwatchVersion = ver_str;
            } 
            else if (toks[0] == "OS") {
                std::string os_version = get_required_token_i(line, 4, 4);
                sysInfo.m_osVersion = os_version;
                db_assert(true, "Found os version = %s\n", os_version.c_str());
            } else {
                db_abort("ERROR: invalid version string: %s\n", line.c_str());
            }
        }
        else if (DOES_LINE_CONTAIN(line, "Start TSC")) {
            pw_u64_t start_tsc = ATOULL(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            sysInfo.m_startTsc = start_tsc;
            db_assert(true, "Found start tsc = %llu\n", TO_ULL(start_tsc));
        }
        else if (DOES_LINE_CONTAIN(line, "Stop TSC")) {
            pw_u64_t stop_tsc = ATOULL(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            sysInfo.m_stopTsc = stop_tsc;
            db_assert(true, "Found stop tsc = %llu\n", TO_ULL(stop_tsc));
        }
        else if (DOES_LINE_CONTAIN(line, "Start Timeval")) {
            pw_u64_t start_timeval = ATOULL(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            sysInfo.m_startTimeval = start_timeval;
            db_assert(true, "Found start timeval = %llu\n", TO_ULL(start_timeval));
        }
        else if (DOES_LINE_CONTAIN(line, "Host Name")) {
            std::string host_name = get_required_token_i(line, 4, 4);
            sysInfo.m_hostName = host_name;
            db_assert(true, "Found host name = %s\n", host_name.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "OS Name")) {
            std::string os_name = get_required_token_i(line, 4, 4);
            sysInfo.m_osName = os_name;
            db_assert(true, "Found os name = %s\n", os_name.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "OS Type")) {
            std::string os_type = get_required_token_i(line, 4, 4);
            sysInfo.m_osType = os_type;
            db_assert(true, "Found os type = %s\n", os_type.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Brand")) {
            std::string cpu_brand = get_required_token_i(line, 4, 4);
            sysInfo.m_cpuBrand = cpu_brand;
            db_assert(true, "Found CPU brand = %s\n", cpu_brand.c_str());
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Family")) {
            sysInfo.m_cpuFamily = atoi(get_required_token_i(line, 4, 4).c_str());
            db_assert(true, "Found CPU Family = %u\n", sysInfo.m_cpuFamily);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Model")) {
            sysInfo.m_cpuModel = atoi(get_required_token_i(line, 4, 4).c_str());
            db_assert(true, "Found CPU model = %u\n", sysInfo.m_cpuModel);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Stepping")) {
            sysInfo.m_cpuStepping = atoi(get_required_token_i(line, 4, 4).c_str());
            db_assert(true, "Found CPU stepping = %u\n", sysInfo.m_cpuStepping);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU C-states Clock Rate")) {
            sysInfo.m_cStateMult = atoi(get_required_token_i(line, 6, 6).c_str());
            db_assert(true, "Found CPU c-states mult = %u\n", sysInfo.m_cStateMult);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Bus Frequency (KHz)")) {
            sysInfo.m_busClockFreq = strtoul(get_required_token_i(line, 6, 6).c_str(), NULL, 10);
            db_assert(true, "Found bus clock freq (KHz) = %llu\n", TO_ULL(sysInfo.m_busClockFreq));
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Bus Frequency (MHz)")) {
            sysInfo.m_busClockFreqMHz = strtof(get_required_token_i(line, 6, 6).c_str(), NULL);
            db_assert(true, "Found bus clock freq (MHz) = %.2f\n", sysInfo.m_busClockFreqMHz);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU Perf Status")) {
            str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
            assert(tokens.size() > 5);
            sysInfo.m_perfBitsLow = atoi(tokens[5].c_str()); sysInfo.m_perfBitsHigh = atoi(tokens[6].c_str());
            db_assert(true, "Low = %u, High = %u\n", sysInfo.m_perfBitsLow, sysInfo.m_perfBitsHigh);
        }
        else if (DOES_LINE_CONTAIN(line, "CPU C-states =")) {
            str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
            assert(tokens.size() > 3);

            m_thread_level_supported_msrs.insert(m_thread_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);
            m_core_level_supported_msrs.insert(m_core_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);
            m_module_level_supported_msrs.insert(m_module_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);
            m_package_level_supported_msrs.insert(m_package_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);

            for (size_t i=3; i<tokens.size(); ++i) {
                db_assert(tokens[i].find('C') == 0, "Error: malformed token = %s\n", tokens[i].c_str());
                int num = atoi(&(tokens[i].c_str()[1]));
                const std::string& type = tokens[++i];
                db_fprintf(stderr, "C%d: Type = %s\n", num, type.c_str());
                if (type == "Package") {
                    m_package_level_supported_msrs[num] = 1;
                } else if (type == "Core") {
                    m_core_level_supported_msrs[num] = 1;
                } else if (type == "Thread") {
                    m_thread_level_supported_msrs[num] = 1;
                } else {
                    assert(false);
                }
            }
            db_assert(true, "Debugging\n");
        }
        else if (DOES_LINE_CONTAIN(line, "Turbo Threshold")) {
            sysInfo.m_turboThreshold = ATOULL(get_required_token_i(line, 4, 4).c_str(), NULL, 10);
            db_assert(true, "Found turbo threshold = %llu\n", TO_ULL(sysInfo.m_turboThreshold));
        }
        else if (DOES_LINE_CONTAIN(line, "Bus clock frequency")) {
            sysInfo.m_busClockFreq= strtoul(get_required_token_i(line, 6, 6).c_str(), NULL, 10);
            db_assert(true, "Found bus clock freq (KHz) = %llu\n", TO_ULL(sysInfo.m_busClockFreq));
        }
        else if (line.find("Num CPUs") != std::string::npos) {
            /*
             * Found # cpus on collection
             * machine.
             */
            int num_cpus = atoi(get_required_token_i(line, 4, 4).c_str());
            sysInfo.m_cpuCount = num_cpus;
            db_assert(true, "Found # cpus = %d\n", num_cpus);
        }
        else if (line.find("CPU Topology") != std::string::npos) {
            sysInfo.m_cpuTopology = line.substr(line.find_first_of("=")+2); // +2 ==> 1 for the '=' and 1 for the space after it.
            str_vec_t toks = Tokenizer(line, " ").get_all_tokens();
            assert(toks.size() > (size_t)3);
            int size = (int)toks.size();
            /*
             * We don't need the first 3 tokens (i.e. "Cpu Topology =")
             */
            std::map <int, int_vec_t> tmp_map;
            int max_core_id = -1, curr_core_id = -1, max_phys_id = -1;
            for (int i=3; i<size; ) {
                str_t tokens[5];
                /*
                 * Format of each logical CPU is:
                 * proc # <space> physical id <space> siblings <space> core id <space> cores <space>"
                 */
                for (int j=0; j<5; ++j, ++i) {
                    tokens[j] = toks[i];
                }
                int proc = atoi(tokens[0].c_str()), phys_id = atoi(tokens[1].c_str()), core_id = atoi(tokens[3].c_str());
                tmp_map[((phys_id << 16) | core_id)].push_back(proc);
                sysInfo.m_abstractPkgMap[phys_id] = phys_id;
                if (phys_id > max_phys_id) {
                    max_phys_id = phys_id;
                }
            }
            sysInfo.m_packageCount = max_phys_id + 1;
            for (std::map <int, int_vec_t>::iterator iter = tmp_map.begin(); iter != tmp_map.end(); ++iter) {
                int_vec_t procs = iter->second;
                int core_id = ++curr_core_id;
                int pkg_id = (iter->first >> 16) & 0xffff, act_core_id = iter->first & 0xffff; // higher 16 bits for pkg, lower 16 bits for core
                db_fprintf(stderr, "pkg_id = %d, core_id = %d, act_core_id = %d\n", pkg_id, core_id, act_core_id);
                sysInfo.m_abstractCoreMap[core_id] = act_core_id;
                for (unsigned i=0; i<procs.size(); ++i) {
                    sysInfo.m_htMap[procs[i]] = core_id;
                    sysInfo.m_abstractThreadMap[procs[i]] = procs[i];
                }
                if (core_id > max_core_id) {
                    max_core_id = core_id;
                }
            }
            sysInfo.m_coreCount = max_core_id + 1;
            {
                int cpu;
                db_fprintf(stderr, "HT map dump...\n");
                for_each_online_cpu(cpu) {
                    db_fprintf(stderr, "%d -> %d\n", cpu, sysInfo.m_htMap[cpu]);
                }
                db_fprintf(stderr, "Thread map dump...\n");
                for_each_online_cpu(cpu) {
                    db_fprintf(stderr, "%d -> %d\n", cpu, sysInfo.m_abstractThreadMap[cpu]);
                }
                db_fprintf(stderr, "Core map dump...\n");
                for_each_online_core(cpu) {
                    db_fprintf(stderr, "%d -> %d\n", cpu, sysInfo.m_abstractCoreMap[cpu]);
                }
                db_fprintf(stderr, "Pkg map dump...\n");
                for (std::map<int,int>::const_iterator citer = sysInfo.m_abstractPkgMap.begin(); citer != sysInfo.m_abstractPkgMap.end(); ++citer) {
                    db_fprintf(stderr, "%d -> %d\n", citer->first, citer->second);
                }
            }
            db_assert(true, "Found CPU Topology=%s\n", sysInfo.m_cpuTopology.c_str());
            db_assert(true, "# cores per package = %d\n", NUM_CORES_PER_PACKAGE());
        }
        else if (line.find("TSC Frequency") != std::string::npos) {
            int tsc_freq = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            sysInfo.m_tscFreq = tsc_freq; // in MHz
            db_assert(true, "Found TSC freq = %d\n", tsc_freq);
            switch (sysInfo.m_cStateMult) {
                case 0: /* Cx MSRs count at TSC freq. */
                    sysInfo.m_cStateMult = 1;
                    break;
                default: /* Cx MSRs count at 'sysInfo.m_cStateMult' KHz. */
                    sysInfo.m_cStateMult = (int) ((1.0 * tsc_freq * 1000) / (1.0 * sysInfo.m_cStateMult));
                    db_assert(true, "tsc freq = %d, cstate mult = %u\n", tsc_freq, sysInfo.m_cStateMult);
                    break;
            }
        }
        else if (line.find("Microcode patch version") != std::string::npos) {
            int micro_patch_ver = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            sysInfo.m_microPatchVer = micro_patch_ver;
            db_assert(true, "FOUND micro patch = %d\n", micro_patch_ver);
        }
        else if (line.find("Any thread bit") != std::string::npos) {
            sysInfo.m_wasAnyThreadSet = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            db_assert(true, "FOUND any-thread bit = %u\n", sysInfo.m_wasAnyThreadSet);
        }
        else if (line.find("Auto demote enabled") != std::string::npos) {
            sysInfo.m_wasAutoDemoteEnabled = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            db_assert(true, "FOUND auto-demote bit = %u\n", sysInfo.m_wasAutoDemoteEnabled);
        }
        else if (line.find("Collection switches") != std::string::npos) {
            sysInfo.m_collectionSwitches = atoi(get_required_token_i(line, 2, 2, "=").c_str());
            db_assert(true, "FOUND collection switches = %u\n", sysInfo.m_collectionSwitches);
        }
        else if (line.find("Total Collection Time") != std::string::npos) {
            /*
             * Found collection time.
             */
            sysInfo.m_collectionTime = get_required_token_i(line, 5, 6);
            db_assert(true, "Found ctime = %s\n", sysInfo.m_collectionTime.c_str());
        }
        else if (line.find("Profiled Application") != std::string::npos) {
            /*
             * Found profiled app details.
             */
            str_vec_t toks = Tokenizer(line, " \t").get_all_tokens();
            assert(toks.size() == (size_t)8);
            sysInfo.m_appPID = atoi(toks[4].c_str());
            sysInfo.m_appProfiled = toks[7];
        }
        else if (line.find("HARDWARE C-STATE Mappings") != std::string::npos) {
            /*
             * Found ACPI <-> H/W C-STATE mapping info.
             */
            /*
             * We're going to be storing the mapping in
             * a vector -- reserve space for it here.
             */
            sysInfo.m_stateMapping.assign(MAX_MSR_ADDRESSES,99); /* Reserve space for 'MAX_MSR_ADDRESSES' slots, writing '99' into each */
            /*
             * This is NOT the LAST section -- iterate
             * until we get to the "TARGET RESIDENCIES" section.
             */
            while(!lines.empty()) {
                line = lines.front(); lines.pop_front();

                if (line.empty() || line.size() == 1) {
                    continue;
                } else if (line.find("TARGET RESIDENCIES") != std::string::npos) {
                    lines.push_front(line);
                    break;
                }
		db_fprintf(stderr, "ACPI line = %s\n", line.c_str());
#if 0
                // Tokenizer tok(line, " \t");
                const char *__tmp_1 = tok.get_next_token();
                assert(__tmp_1 != NULL);
                int ac = atoi(__tmp_1);
                // int ac = atoi(GET_NEXT_TOKEN(tok));
                const char *__tmp_2 = tok.get_next_token();
                assert(__tmp_2 != NULL);
                str_t hws = __tmp_2;
                // str_t hws = GET_NEXT_TOKEN(tok); // tok.get_next_token();
#else
		str_vec_t tmp_toks = Tokenizer(line, " \t").get_all_tokens();
		db_assert(tmp_toks.size() == (size_t)2, "Error: # tokens found = %lu, required = 2\n", TO_UL(tmp_toks.size()));
		int ac = atoi(tmp_toks[0].c_str());
		str_t hws = tmp_toks[1];
#endif
                size_t endpos = hws.rfind('C');
                db_assert(endpos != std::string::npos, "MALFORMED ACPI mapping line!\n");
                /*
                 * Everything from 'endpos+1' to EOL
                 * is the H/W C-state number.
                 */
                int hw = atoi(&(hws.c_str())[++endpos]);
                sysInfo.m_stateMapping[ac] = hw;
                db_assert(true, "%d <-> %d\n", ac, hw);
            }
            std::vector<int> *vec = &sysInfo.m_stateMapping;
            db_copy(vec->begin(), vec->end(), std::ostream_iterator<int>(std::cerr, "\n"));
            db_assert(true, "TODO\n");
        }
        else if (line.find("TARGET RESIDENCIES") != std::string::npos) {
            while (!lines.empty()) {
                line = lines.front(); lines.pop_front();

                if (line.empty() || line.size() == 1) {
                    continue;
                } else if (line.find("AVAILABLE FREQUENCIES") != std::string::npos) {
                    lines.push_front(line);
                    break;
                }

                str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
                assert(tokens.size() == 3);
                int state = atoi(&(tokens[0].c_str())[1]);
                // sysInfo.m_targetRes[state] = (atoi(tokens[2].c_str())/1000);
                sysInfo.m_targetRes[state] = (atoi(tokens[2].c_str()));
            }

        }
        else if (DOES_LINE_CONTAIN(line, "AVAILABLE FREQUENCIES")) {
            str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
            assert(tokens.size() > 3);
            for (size_t i=3; i<tokens.size(); ++i) {
                m_availableFrequenciesKHz.push_back(atoi(tokens[i].c_str()));
            }
            sysInfo.m_availableFrequenciesKHz.insert(sysInfo.m_availableFrequenciesKHz.begin(), m_availableFrequenciesKHz.begin(), m_availableFrequenciesKHz.end());
            db_copy(m_availableFrequenciesKHz.begin(), m_availableFrequenciesKHz.end(), std::ostream_iterator<u32>(std::cerr, "\n"));
            db_assert(true, "Found available freqs list! Size = %lu\n", TO_UL(m_availableFrequenciesKHz.size()));
        }
        else if (DOES_LINE_CONTAIN(line, "TOTAL SAMPLES")) {
            str_vec_t tokens = Tokenizer(line, " ").get_all_tokens();
            sysInfo.m_totalSamples = (pw_u32_t)atoi(tokens[3].c_str());
            sysInfo.m_droppedSamples = (pw_u32_t)atoi(tokens[7].c_str());
            assert(tokens.size() == 8);
            db_assert(true, "Total # samples = %lu, # dropped samples = %lu\n", TO_UL(sysInfo.m_totalSamples), TO_UL(sysInfo.m_droppedSamples));
        }
        else if (line.find("DESCENDENT PIDS LIST") != std::string::npos) {
            while (!lines.empty()) {
                std::string::size_type idx = -1;
                line = lines.front();
                lines.pop_front();
                if (!line.size() || line[0] == '\n') {
                    continue;
                }
                if ((idx = line.find("PID")) == std::string::npos || idx != 0) {
                    // Different section?
                    lines.push_front(line);
                    break;
                }
                str_t pid_str = line.substr(4);
                db_fprintf(stderr, "Pid substr = %s\n", pid_str.c_str());
                sysInfo.m_descendentPids.push_back(pid_str);
            }
            db_copy(sysInfo.m_descendentPids.begin(), sysInfo.m_descendentPids.end(), std::ostream_iterator<std::string>(std::cerr, " "));
        }
    }
    /*
     * Older (i.e. pre v310) files don't contain some information we need. Patch
     * up the required info as best we can.
     * Note that we only really need to worry about Saltwell cores here because the
     * only time we'll be analyzing older ".ww1" files is when we're importing that
     * data into AXE (per Bob's request, wuwatch itself isn't backwards compatible).
     * -------------------------------------------------
     * TODO: Check this!!!
     * -------------------------------------------------
     */
    if (WUWATCH_VERSION(m_driverMajor, m_driverMinor, m_driverOther) < WUWATCH_VERSION(3, 1, 0)) {
        assert(sysInfo.m_cStateMult == 1); // Sanity!
        /*
         * Add information on the Cx MSR clock rate, bus frequency , the IA32_PERF_STATUS bits, supported C-states, 
         * whether the any-thread bit is set and whether auto-demote was enabled.
         * NOTE: the last two (any-thread and auto-demote) are NOT arch specific! We make the following
         * assumptions:
         * 1. That any-thread is ALWAYS ZERO!
         * 2. That auto-demote is ALWAYS DISABLED! This is a relatively safe assumption: all Android distros
         * seem to have auto-demote disabled; also it's disabled in newer mainline Linux kernels (newer than 2.6.35).
         */
        m_thread_level_supported_msrs.insert(m_thread_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);
        m_core_level_supported_msrs.insert(m_core_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);
        m_module_level_supported_msrs.insert(m_module_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);
        m_package_level_supported_msrs.insert(m_package_level_supported_msrs.end(), MAX_MSR_ADDRESSES, -1);

        sysInfo.m_wasAnyThreadSet = sysInfo.m_wasAutoDemoteEnabled = 0;
        /*
         * For ALL archs:
         * C0 == Thread-level; C1 == Core-level;
         */
        m_thread_level_supported_msrs[MPERF] = 1; m_core_level_supported_msrs[APERF] = 1;

        if (PW_IS_SALTWELL(sysInfo.m_cpuModel)) {
            sysInfo.m_cStateMult = 995; // Cx MSRs on Saltwell count at 995 KHz
            sysInfo.m_cStateMult = (int) ((1.0 * sysInfo.m_tscFreq * 1000) / (1.0 * sysInfo.m_cStateMult));
            db_assert(true, "tsc freq = %d, cstate mult = %u\n", sysInfo.m_tscFreq, sysInfo.m_cStateMult);

            sysInfo.m_busClockFreq = 100000; // 100 MHz
            sysInfo.m_perfBitsLow = 8; sysInfo.m_perfBitsHigh = 12;
            /*
             * For saltwell:
             * C2, C4, C5, C6 are PACKAGE-level
             * However, C5 is intel-private, so we only include it if we've been told to do so.
             */
            m_package_level_supported_msrs[C2] = m_package_level_supported_msrs[C4] = m_package_level_supported_msrs[C6] = 1;
#if IS_INTEL_INTERNAL
            m_package_level_supported_msrs[C5] = 1;
#endif
        } else {
            /*
             * (Sigh!) Need to do this until we can confirm we're not supporting
             * backwards compatibility!
             * For big-core:
             * C3, C6 (and possibly C7) are CORE-level
             */
            /*
             * Note: nothing to do for the Cx clock rate: the code that handles TSC frequency
             * calculations would have auto-set this for us.
             */
            m_core_level_supported_msrs[C3] = m_core_level_supported_msrs[C6] = 1;
            switch (sysInfo.m_cpuModel) {
                case 0x1a:
                case 0x1e:
                case 0x1f:
                case 0x25:
                case 0x2c:
                case 0x2e:
                    /*
                     * NHM/WMR
                     */
                    sysInfo.m_busClockFreq = 133333;
                    sysInfo.m_perfBitsLow = 0; sysInfo.m_perfBitsHigh = 7;
                    break;
                case 0x2a:
                case 0x2d:
                case 0x3a:
                    /*
                     * SNB/IVB
                     */
                    sysInfo.m_busClockFreq = 100000;
                    sysInfo.m_perfBitsLow = 8; sysInfo.m_perfBitsHigh = 15;
                    m_core_level_supported_msrs[C7] = 1;
                    break;
                default:
                    fprintf(stderr, "ERROR: unknown arch = %d.%d.%d!\n", sysInfo.m_cpuFamily, sysInfo.m_cpuModel, sysInfo.m_cpuStepping);
                    assert(false);
            }
        }
        // assert(false);
    }

    if (PW_IS_SALTWELL(sysInfo.m_cpuModel)) {
        if (PW_IS_MFD(sysInfo.m_cpuModel)) {
            max_lss_num_in_nc = MFD_MAX_LSS_NUM_IN_NC;
            max_lss_num_in_sc = MFD_MAX_LSS_NUM_IN_SC;
        } else if (PW_IS_CLV(sysInfo.m_cpuModel)) {
            max_lss_num_in_nc = CLV_MAX_LSS_NUM_IN_NC;
            max_lss_num_in_sc = CLV_MAX_LSS_NUM_IN_SC;
        }
    }

    /*
     * Also populate the timeline version.
     */
    /*
    char wudump_ver[100];
    sprintf(wudump_ver, "%d.%d.%d", WUWATCH_VERSION_VERSION, WUWATCH_VERSION_INTERFACE, WUWATCH_VERSION_OTHER);
    sysInfo.m_wudumpVersion = wudump_ver;
    */

    return PW_SUCCESS;
};


void pwr::WuParser::post_process_samples_i(sample_list_t& samples, sample_type_t type)
{
    /*
     * Post processing only valid for 'S_RES', 'D_RES' and 'W_STATE' samples.
     */
    switch (type) {
        case S_RESIDENCY:
        case D_RESIDENCY:
        case W_STATE:
            break;
        default:
            return;
    }
    for (sample_list_t::iterator iter = samples.begin(); iter != samples.end(); ++iter) {
        PWCollector_sample_t& sample = *iter;
        /*
         * Specific actions depend on sample types.
         */
        assert(sample.sample_type == type);
        switch (type) {
            case S_RESIDENCY:
                /*
                 * Calculate the delta for S residency counters
                 * Will do this calculation inside the power driver in future release
                 */
                {
                    pw_u64_t temp_res[6];

                    memcpy(temp_res, sample.s_residency_sample.data, sizeof(temp_res));

                    sample.s_residency_sample.data[0] -= prev_s_res[0];

                    if (!m_do_convert_s_d_res_to_ticks) {
                        sample.s_residency_sample.data[0] /= sysInfo.m_tscFreq; // convert from '# ticks' to 'usecs' for wudump output
                    }

                    for (int i=1; i<4; ++i) {
                        sample.s_residency_sample.data[i] -= prev_s_res[i];
                        if (m_do_convert_s_d_res_to_ticks) {
                            sample.s_residency_sample.data[i] *= sysInfo.m_tscFreq; // usecs * MHz == # ticks
                        }
                    }

                    if (!m_do_convert_s_d_res_to_ticks) {
                        sample.s_residency_sample.data[4] /= sysInfo.m_tscFreq; // convert from '# ticks' to 'usecs' for wudump output
                    }

                    if (sample.s_residency_sample.data[4] > 0) {
                        sample.s_residency_sample.data[4] = sample.s_residency_sample.data[3];
                        sample.s_residency_sample.data[3] = 0;
                    }

                    pw_u64_t totalSx = 0;
                    for (int i=1; i<5; ++i) {
                        totalSx += sample.s_residency_sample.data[i];
                    }

                    if (totalSx <= sample.s_residency_sample.data[0]) {
                        sample.s_residency_sample.data[0] -= totalSx;
                    } else {
                        sample.s_residency_sample.data[0] = 0;
                    }

                    memcpy(prev_s_res, temp_res, sizeof(temp_res));
                    // m_output_samples.push_back(sample);
                }
                break;
            case D_RESIDENCY:
                /* 
                 * Calculate the delta for D residency counters
                 * Will do this calculation insider the power driver in future release
                 */ 
                {
                    const u16 *mask = sample.d_residency_sample.mask;
                    pw_u32_t num = sample.d_residency_sample.num_sampled;

                    if (sample.d_residency_sample.device_type == PW_SOUTH_COMPLEX) {

                        for (pw_u32_t idx=0; idx<num; idx++) { // from 0 --> 2
                            pw_u32_t id = mask[idx];
                            if (id < max_lss_num_in_sc) {
                                pw_u64_t temp_res[4];

                                memcpy(temp_res, sample.d_residency_sample.d_residency_counters[idx].data, sizeof(temp_res));

                                sample.d_residency_sample.d_residency_counters[idx].data[0] -= prev_d_res[id][0];
                                if (m_do_convert_s_d_res_to_ticks) {
                                    sample.d_residency_sample.d_residency_counters[idx].data[0] *= sysInfo.m_tscFreq; // usecs * MHz == # ticks
                                } else {
                                    sample.d_residency_sample.d_residency_counters[idx].data[0] /= 1000; // convert from 'usecs' to 'msecs' for compatibility with other fields
                                }
                                for (int i=1; i<4; ++i) {
                                    sample.d_residency_sample.d_residency_counters[idx].data[i] -= prev_d_res[id][i];
                                    if (m_do_convert_s_d_res_to_ticks) {
                                        sample.d_residency_sample.d_residency_counters[idx].data[i] *= (sysInfo.m_tscFreq * 1000); // msecs * KHz == # ticks
                                    }
                                }

                                memcpy(prev_d_res[id], temp_res, sizeof(temp_res));
                            }
                        }
                    }
                    // m_output_samples.push_back(sample);
                }
                break;
            case W_STATE:
                /*
                 * Convert the timeout in msec to tsc unit
                 */
                sample.w_sample.expires = sample.tsc + sample.w_sample.expires * sysInfo.m_tscFreq;
                break;
            default: /* S_STATE, D_STATE etc. */
                break;
        }
    }
};

/*
 * INTERNAL API:
 * A function to read samples from the
 * binary file and check that they meet
 * sorting requirements.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_read_i(void)
{
    /*
     * We have a combined input file, containing driver output
     * and also system configuration information.
     */
    FILE *in_fp = NULL;
 
#if defined(_WIN32)
    if (fopen_s(&in_fp, m_combined_input_file_name.c_str(), "rb") != 0) {
        in_fp = NULL;
    } 
#else
    in_fp = fopen(m_combined_input_file_name.c_str(), "rb");
#endif
    if(!in_fp){
        db_perror("fopen error");
        return -PW_ERROR;
    }
    u64 driver_beg_off = sizeof(u64), driver_end_off = 0;
    /*
     * Step (1): read the 'sys_params_found.txt'
     * file.
     */
    {
        if (get_sys_params_i(in_fp, driver_end_off)) {
            db_fprintf(stderr, "ERROR retrieving sys params!\n");
            return -PW_ERROR;
        }
    }

    db_fprintf(stderr, "# CPUS = %d\n",NUM_ONLINE_CPUS());

    /*
     * Sanity tests!
     */
    assert(driver_beg_off <= driver_end_off);
    db_assert(sysInfo.m_cpuCount > 0, "ERROR: # cpus = %d\n", sysInfo.m_cpuCount);

    // sample_vec_t samples(NUM_MSGS_TO_READ + 1);
    //int cpu = 0;

    /*
     * Step (2): read (driver) samples from disk into (per-cpu) output 
     * vectors.
     */
    /*
     * First, we 'seek' back to the start of the driver data.
     */
    if (fseek(in_fp, (long)driver_beg_off, SEEK_SET)) {
        db_perror("fseek error in do_read_i()");
        return -PW_ERROR;
    }

    memset(prev_s_res, 0, sizeof(prev_s_res));
    memset(prev_d_res, 0, sizeof(pw_u64_t)*MAX_LSS_NUM_IN_SC*4);

    db_fprintf(stderr, "Driver major.minor.other = %d.%d.%d\n", sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther);
    /*
     * OK, how we read in the samples depends on which version of the driver we're communicating
     * with. Older drivers (i.e. pre 3.1.0) would return fixed, 128 byte 'PWCollector_sample' instances.
     * Newer versions (i.e. >= 3.1.0) return variable-length 'PWCollector_msg' instances.
     */
    if (WUWATCH_VERSION(sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther) >= WUWATCH_VERSION(3, 1, 0)) {
        /*
         * Newer version: read in data and convert to PWCollector_sample instances before
         * attempting to process them.
         */
        pw_u32_t bytes_left = (u32)(driver_end_off - driver_beg_off);
        std::vector <char> data_buffer(65536);
        u64 num_samples = 0; //, total_sample_size = 0;
        int buffer_offset = 0;
        db_fprintf(stderr, "Beginning parsing, # orig samples = %lu\n", TO_UL(m_origSamples.size()));
        while (bytes_left) {
            char *read_ptr = &data_buffer[buffer_offset];
            size_t read_len = std::min((pw_u32_t)(65536 - buffer_offset), bytes_left);
            size_t count = fread(read_ptr, sizeof(char), read_len, in_fp);
            db_fprintf(stderr, "Count = %lu, bytes left = %lu\n", TO_UL(count), TO_UL(bytes_left));
            if (count == 0) {
                /*
                 * 'fread()' doesn't distinguish between EOF and ERROR. We could manually check
                 * (by using 'feof()'), but in our case it doesn't matter because we
                 * shouldn't be getting an EOF anyway (since the 'sys params' section is encoded
                 * AFTER any driver data, which means we should never EOF if we're reading in
                 * only the driver data).
                 */
                perror("fread error");
                return -PW_ERROR;
            }
            bytes_left -= (int)count;
            count += buffer_offset;
            buffer_offset = 0;

            if (do_parse_messages_i(data_buffer, count, m_origSamples, buffer_offset, num_samples)) {
                db_fprintf(stderr, "ERROR parsing messages\n");
                return -PW_ERROR;
            }
        }
        db_fprintf(stderr, "# orig samples = %lu, # samples from parser = %llu\n", TO_UL(m_origSamples.size()), (unsigned long long)num_samples);
    } else {
        pw_u64_t num_driver_output_samples = (driver_end_off - driver_beg_off) / sizeof(PWCollector_sample_t);
        size_t num_samples_to_read = 0, curr_idx = 0;
        size_t num_read = 0;
        db_fprintf(stderr, "DRIVER OUTPUT SIZE = %llu\n", TO_ULL(num_driver_output_samples));

        m_origSamples = sample_vec_t((unsigned int)num_driver_output_samples + 1);

        /*
         * Then we read in the driver samples.
         */
        while (num_driver_output_samples > 0) {
            num_samples_to_read = (size_t)std::min(num_driver_output_samples, (pw_u64_t)4096); // read a max of 4096 samples at a time
            num_read = fread(&m_origSamples[curr_idx], sizeof(PWCollector_sample_t), num_samples_to_read, in_fp);
            if (num_read < num_samples_to_read) {
                db_perror("fread error while reading driver samples");
                return -PW_ERROR;
            }
            num_driver_output_samples -= num_read;
            curr_idx += num_read;
        }
    }
    /*
     * We don't need to read anything else from the file.
     */
    fclose(in_fp);

    return PW_SUCCESS;
};

void augment_tpf_samples_i(std::vector <PWCollector_sample_t>& samples)
{
    std::map <int, pw_u32_t> __per_thread_req_freq_map, __per_core_act_freq_map;
    std::map <int, pw_u64_t> __per_core_prev_tsc_map;
    for (std::vector<PWCollector_sample_t>::reverse_iterator riter = samples.rbegin(); riter != samples.rend(); ++riter) {
        if (riter->sample_type == P_STATE) {
            int cpuidx = riter->cpuidx, coreid = GET_CORE_GIVEN_LCPU(riter->cpuidx);

            riter->p_sample.unhalted_core_value = (pw_u64_t)(__per_thread_req_freq_map[cpuidx]) << 32 | (pw_u64_t)__per_core_act_freq_map[coreid];

            if (__per_core_prev_tsc_map[coreid]) {
                riter->p_sample.unhalted_ref_value = __per_core_prev_tsc_map[coreid] - riter->tsc;
            }

            __per_thread_req_freq_map[cpuidx] = riter->p_sample.prev_req_frequency;
            __per_core_act_freq_map[coreid] = riter->p_sample.frequency;
            __per_core_prev_tsc_map[coreid] = riter->tsc;
        }
    }
};
/*
 * INTERNAL API:
 * The meat of the algorithm. Sort (previously read) samples, 
 * create TPS groupings, measure Cx residencies, determine
 * wakeup causes etc.
 *
 * @returns: 0 on success, -1 on error
 */
int pwr::WuParser::do_parse_i(void)
{
    /*
     * (0) Perform any initialization etc.
     */
    {
        /*
         * Check if we need to convert S, D residency values to TSC ticks.
         */
        if (!m_do_convert_s_d_res_to_ticks && m_is_from_axe) {
            /*
             * OK, we've been called by AXE -- convert residency to TSC ticks
             * (but ONLY if the driver version >= 3.0.1)
             */
            if (WUWATCH_VERSION(sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther) >= WUWATCH_VERSION(3, 0, 1)) {
                db_fprintf(stderr, "driver major = %d, minor = %d, other = %d, version = %u\n", sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther, WUWATCH_VERSION(sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther));
                m_do_convert_s_d_res_to_ticks = true;
            }
        }
    }

    /*
     * (1) Build topology.
     */
    build_topology_i();
    /*
     * (2) Set various MSRs.
     */
    {
        set_thread_level_msrs_i(m_thread_level_supported_msrs);
        set_core_level_msrs_i(m_core_level_supported_msrs);
        // set_module_level_msrs_i(m_module_level_supported_msrs);
        set_package_level_msrs_i(m_package_level_supported_msrs);
    }
    /*
     * (3) Sort input samples.
     * Update: make sure we do a STABLE sort!
     */
    // std::sort(m_origSamples.begin(), m_origSamples.end());
    std::stable_sort(m_origSamples.begin(), m_origSamples.end());
    /*
     * (3 a): dump them, if instructed to do so.
     */
    if (m_do_dump_orig_samples) {
        if (m_combined_input_file_name.size()) {
            std::string __name = m_combined_input_file_name + ".org";
            std::ofstream stream(__name.c_str());
            std::copy(m_origSamples.begin(), m_origSamples.end(), std::ostream_iterator<PWCollector_sample_t>(stream, ""));
        } else {
            std::copy(m_origSamples.begin(), m_origSamples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
        }
    }
    /*
     * 3(b) perform TPF modifications.
     */
    {
        augment_tpf_samples_i(m_origSamples);
    }
    /*
     * (4) Replay trace.
     */
    replay_trace_i(m_origSamples);
    return PW_SUCCESS;
};

bool check_sorted(const sample_list_t& samples)
{
    pw_u64_t prev_tsc = 0;
    for (sample_list_t::const_iterator citer = samples.begin(); citer != samples.end(); ++citer) {
        if (prev_tsc && citer->tsc < prev_tsc) {
            fprintf(stderr, "UNSORTED: TSC = %llu\n", citer->tsc);
            return false;
        }
        prev_tsc = citer->tsc;
    }
    return true;
};

/*
 * INTERNAL API:
 * Perform finalizations for all (previously parsed) samples, sort them and collate
 * them into a single output list in pareparation for returning them to the caller(s).
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_finalize_and_collate_i(void)
{
    /*
     * First, finalize results.
     */
    std::map <sample_type_t, sample_list_t> samples;
    finalize_samples_i(samples);
    for (int i=C_STATE; i<SAMPLE_TYPE_END; ++i) {
        db_fprintf(stderr, "[%s]: %llu\n", s_long_sample_names[i], TO_ULL(samples[(sample_type_t)i].size()));
    }
    /*
     * Finally, collate all samples into a single (sorted) list.
     */
    for (int i=C_STATE; i<SAMPLE_TYPE_END; ++i) {
        m_output_samples.insert(m_output_samples.end(), samples[(sample_type_t)i].begin(), samples[(sample_type_t)i].end());
        // m_output_samples.merge(samples[(sample_type_t)i]);
    }
    m_output_samples.sort();
    /*
    */
    // if (g_do_debugging) {
    if (false) {
        std::cerr << "---BEGIN-DUMP-RESULTS---" << std::endl;
        std::copy(m_output_samples.begin(), m_output_samples.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
        std::cerr << "---END-DUMP-RESULTS---" << std::endl;
    }
    /*
     * Print some debug information on #overcounts.
     */
    if (m_do_dump_sample_stats) {
        fprintf(stderr, "Total # samples = %u, # dropped samples = %u\n", sysInfo.m_totalSamples, sysInfo.m_droppedSamples);
        fprintf(stderr, "Total # PKG CX samples = %d, # PKG CX overcounts = %d\n", s_num_pkg_cx_samples, s_num_pkg_cx_overcounts);
    }

    return PW_SUCCESS;
};

/*
 * INTERNAL API:
 * Parse each sample supplied by wuwatch and/or the power driver.
 *
 * @trace: the list of samples to parse.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::replay_trace_i(const std::vector <PWCollector_sample_t>& trace)
{
    //double elapsed = 0.0;
    size_t s = trace.size();
    for (size_t i = 0, j = 1; i < s; ++i, ++j) {
        if (trace[i].sample_type == C_STATE && trace[j].sample_type == C_STATE && trace[i].cpuidx == trace[j].cpuidx && trace[i].tsc == trace[j].tsc) {
            db_fprintf(stderr, "Warning: [%d] has multiple samples with the same TSC (%llu)!\n", trace[i].cpuidx, trace[i].tsc);
            continue;
        }
        const PWCollector_sample_t *sample = &trace[i];
        Processor *thread = NULL;
        if (likely(sample->sample_type != SCHED_SAMPLE)) {
            thread = m_threads[sample->cpuidx];
        } else {
            // TODO: order of 'SCHED' samples should actually depend on the value of the associated 'tps_epoch' field and NOT on the TSC value!!!
            // assert(false);
            thread = m_threads[(const int)sample->e_sample.data[1]];
        }
        if (thread->handle_sample(sample)) {
            db_fprintf(stderr, "ERROR handling sample in replay trace\n");
            return -PW_ERROR;
        }
    }
    return PW_SUCCESS;
};

/*
 * INTERNAL API:
 * Set the MSRs that are applicable only to the individual logical CPUs.
 *
 * @thread_msrs: the list of thread MSRs.
 */
void pwr::WuParser::set_thread_level_msrs_i(const std::vector <int>& thread_msrs)
{
    set_msrs_i(thread_msrs, m_threads);
};

/*
 * INTERNAL API:
 * Set the core-level MSRs, if any.
 *
 * @core_msrs: the list of core MSRs.
 */
void pwr::WuParser::set_core_level_msrs_i(const std::vector <int>& core_msrs)
{
    set_msrs_i(core_msrs, m_cores);
};

/*
 * INTERNAL API:
 * Set the package-level MSRs, if any.
 *
 * @thread_msrs: the list of thread MSRs.
 */
void pwr::WuParser::set_package_level_msrs_i(const std::vector <int>& package_msrs)
{
    set_msrs_i(package_msrs, m_packages);
};

void pwr::WuParser::finalize_samples_i(std::map<sample_type_t, sample_list_t>& samples)
{
    pw_u64_t collection_start_tsc = 0, collection_stop_tsc = 0;
    /*
     * All C-state samples MUST be CORE samples (regardless of whether we're reading
     * Package C-state MSRs). Push all of the package samples (if any) to the cores.
     */
    for (unsigned i=0; i<m_packages.size(); ++i) {
        m_packages[i]->push_samples_to_cores();
    }
    /*
     * Then, extract information on collection {start, stop} TSC values.
     */
    for (unsigned i=0; i<m_packages.size(); ++i) {
        pw_u64_t child_min_tsc = 0, child_max_tsc = 0, child_min_mwait_tsc = 0, child_max_mwait_tsc = 0;
        m_packages[i]->set_min_max_collection_tscs(child_min_tsc, child_max_tsc, child_min_mwait_tsc, child_max_mwait_tsc);
        if (child_min_tsc == 0) {
            db_fprintf(stderr, "WARNING: no child min tsc for %s?!\n", m_packages[i]->name().c_str());
            continue;
        }
        assert(child_min_tsc > 0);
        if (collection_start_tsc == 0 || child_min_tsc < collection_start_tsc) {
            collection_start_tsc = child_min_tsc;
        }
        /*
        collection_stop_tsc = std::max(collection_stop_tsc, child_max_mwait_tsc);
        if (collection_stop_tsc == 0) {
            db_fprintf(stderr, "Warning: %s[%d]: child_max_mwait_tsc = %llu, using child_max_tsc = %llu\n", "Package", i, child_max_mwait_tsc, child_max_tsc);
            collection_stop_tsc = child_max_tsc;
        }
        */
        collection_stop_tsc = std::max(collection_stop_tsc, child_max_tsc);
    }
    if (WAS_COLLECTION_SWITCH_SET(sysInfo.m_collectionSwitches, PW_POWER_C_STATE)) {
        if (likely(collection_start_tsc && collection_stop_tsc)) {
            m_total_collection_time = (collection_stop_tsc - collection_start_tsc);
            double collection_time_secs = (double)m_total_collection_time / ((double)sysInfo.m_tscFreq /* MHz */ * 1e6);
            char tmp[10];
            SNPRINTF(tmp, sizeof(tmp), "%.2f", collection_time_secs);
            sysInfo.m_collectionTime = tmp;
        }
    }
    db_fprintf(stderr, "Collection START tsc = %llu, STOP tsc = %llu, total collection time = %llu, sysInfo value = %s\n", collection_start_tsc, collection_stop_tsc, m_total_collection_time, sysInfo.m_collectionTime.c_str());
    /*
     * TPS and TPF samples may need to be pruned/adjusted based on collection start/stop TSCs, but
     * other types don't. This is why we have specialized functions to finalize TPS/TPF samples, but
     * a generic "gather" operation for all other sample types.
     */
    for (unsigned i=0; i<m_packages.size(); ++i) {
        m_packages[i]->finalize_tps_samples(samples[C_STATE], collection_start_tsc, collection_stop_tsc);
        m_packages[i]->finalize_tpf_samples(samples[P_STATE], collection_start_tsc, collection_stop_tsc);
        for (int j=K_CALL_STACK; j<SAMPLE_TYPE_END; ++j) {
            sample_type_t type = (sample_type_t)j;
            m_packages[i]->finalize_other_samples(samples[type], type);
            /*
             * Perform any "post-finalization" processing required here.
             * Currently, only S and D-residency and W-state samples require this 
             * post-processing.
             */
            post_process_samples_i(samples[type], type);
        }
        /*
         * We also need to do some book-keeping for CLTP samples.
         */
        {
            if (PW_IS_CLV(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
                std::list <PWCollector_sample_t>& my_samples = samples[C_STATE];
                for (std::list<PWCollector_sample_t>::iterator curr_core_iter = my_samples.begin(); curr_core_iter != my_samples.end(); ++curr_core_iter) {
                    pw_u32_t __prev_state = curr_core_iter->c_sample.prev_state, __req_state = GET_REQ_FROM_PREV(__prev_state), __act_state = GET_ACT_FROM_PREV(__prev_state);
                    if (__act_state == APERF) {
                        pw_u64_t& __c1_res = RES_COUNT(curr_core_iter->c_sample, APERF); 
                        if (__req_state > APERF) {
                            switch (curr_core_iter->c_sample.break_type) {
                                // case PW_BREAK_TYPE_N:
                                case PW_BREAK_TYPE_A:
                                    break;
                                default:
                                    /*
                                     * We need to tell AXE of 'CI1/C1*' samples. We do so by setting the highest bit in 
                                     * the 'C1' res count (we assume C1 residency will always be less than 0x7fffffffffffffff ticks).
                                     */
                                    SET_C1I_FLAG(__c1_res);
                                    db_fprintf(stderr, "%llu -> %llu, %s\n", curr_core_iter->tsc, __c1_res, GET_BOOL_STRING(IS_C1I_FLAG_SET(RES_COUNT(curr_core_iter->c_sample, APERF))));
                                    // assert(false);
                                    break;
                            }
                        } else {
                            /*
                             * A valid 'C1' sample -- make sure we reset the MSB!
                             */
                            RESET_C1I_FLAG(__c1_res);
                        }
                    }
                }
            }
        }
    }
};
/*
 * INTERNAL API:
 * Helper function to traverse the underlying CPU topology and dump out information about the
 * individual nodes in the topology graph.
 */
void pwr::WuParser::traverse_i()
{
    for (int i=0; i<m_packageCount; ++i) {
        m_packages[i]->dfs_traverse(0);
    }
};

/*
 * INTERNAL API:
 * Helper function to set MSRs for the various nodes in a CPU topology.
 *
 * @msrs: the list of supported MSRs.
 * @which: the list of nodes on which to set the MSRs.
 */
void pwr::WuParser::set_msrs_i(const std::vector<int>& msrs, std::map <int, Processor *>& which)
{
    Processor *proc = NULL;
    FOR_EACH_PROC(proc, which) {
        proc->set_msrs(msrs);
    }
};

/*
 * INTERNAL API:
 * Construct the graph structure depicting the CPU topology of the target system.
 */
void pwr::WuParser::build_topology_i(void)
{
    std::string top_str = sysInfo.m_cpuTopology;
    assert(top_str != "");
    db_fprintf(stderr, "Top str = %s\n", top_str.c_str());
    std::stringstream stream(top_str);
    std::vector <std::string> toks;
    std::string token;
    while (stream >> token) {
        toks.push_back(token);
    }
    int size = (int)toks.size();
    std::map <int, int_vec_t> tmp_map;
    std::map <int, int> threadToCoreMap;
    std::map <int, int> coreToPackageMap;
    int max_core_id = -1, curr_core_id = -1, max_phys_id = -1;
    for (int i=0; i<size; ) {
        std::string tokens[5];
        /*
         * Format of each logical CPU is:
         * proc # <space> physical id <space> siblings <space> core id <space> cores <space>"
         */
        for (int j=0; j<5; ++j, ++i) {
            tokens[j] = toks[i];
        }
        int proc = atoi(tokens[0].c_str()), phys_id = atoi(tokens[1].c_str()), core_id = atoi(tokens[3].c_str());
        m_threadCount++;
        tmp_map[((phys_id << 16) | core_id)].push_back(proc);
        if (phys_id > max_phys_id) {
            max_phys_id = phys_id;
        }
    }
    m_packageCount = max_phys_id + 1;
    for (std::map <int, int_vec_t>::iterator iter = tmp_map.begin(); iter != tmp_map.end(); ++iter) {
        int_vec_t procs = iter->second;
        int core_id = ++curr_core_id;
        int package_id = (iter->first >> 16);
        coreToPackageMap[core_id] = package_id;
        for (unsigned i=0; i<procs.size(); ++i) {
            threadToCoreMap[procs[i]] = core_id;
        }
        if (core_id > max_core_id) {
            max_core_id = core_id;
        }
    }
    m_coreCount = max_core_id + 1;
    {
        db_fprintf(stderr, "Thread map dump...\n");
        /*
        for (int cpu=0; cpu<m_threadCount; ++cpu) {
            db_fprintf(stderr, "%d -> %d\n", cpu, threadToCoreMap[cpu]);
        }
        */
        for (std::map<int,int>::const_iterator citer = threadToCoreMap.begin(); citer != threadToCoreMap.end(); ++citer) {
            db_fprintf(stderr, "%d -> %d\n", citer->first, citer->second);
        }
        db_fprintf(stderr, "Core map dump...\n");
        /*
        for (int core=0; core<m_coreCount; ++core) {
            db_fprintf(stderr, "%d -> %d\n", core, coreToPackageMap[core]);
        }
        */
        for (std::map<int,int>::const_iterator citer = coreToPackageMap.begin(); citer != coreToPackageMap.end(); ++citer) {
            db_fprintf(stderr, "%d -> %d\n", citer->first, citer->second);
        }
    }
    /*
    for (int i=0; i<m_packageCount; ++i) {
        m_packages.push_back(new Package(i));
    }
    for (int i=0; i<m_coreCount; ++i) {
        m_cores.push_back(new Core(i, m_packages[coreToPackageMap[i]]));
    }
    for (int i=0; i<m_threadCount; ++i) {
        m_threads.push_back(new Thread(i, m_cores[threadToCoreMap[i]]));
    }
    */
    assert(m_packageCount < 65536); // satisfy Klockwork!
    for (int i=0; i<m_packageCount; ++i) {
        m_packages[i] = new Package(i);
    }
    for (std::map<int,int>::const_iterator citer = coreToPackageMap.begin(); citer != coreToPackageMap.end(); ++citer) {
        m_cores[citer->first] = new Core(citer->first, m_packages[citer->second]);
    }
    for (std::map<int,int>::const_iterator citer = threadToCoreMap.begin(); citer != threadToCoreMap.end(); ++citer) {
        m_threads[citer->first] = new Thread(citer->first, m_cores[citer->second]);
    }

    for (int i=0; i<m_packageCount; ++i) {
        m_packages[i]->dfs_traverse(0);
    }

    // assert(false);
};

/*
 * INTERNAL API:
 * Convert raw byte data returned by the power driver into 'PWCollector_sample' instances.
 *
 * @data_buffer: the data to convert.
 * @count: the size of data in 'data_buffer'
 * @samples: the list of (newly created) PWCollector sample instances corresponding to the raw data in 'data_buffer'
 * @buffer_offset: offset into 'data_buffer' at which the NEXT read should start populating data; used ONLY if we didn't retrieve a full sample
 *                 from the driver.
 * @num_samples: the number of samples in 'samples'
 *
 * @returns: 0 on success, -1 on error
 */
int pwr::WuParser::do_parse_messages_i(std::vector<char>& data_buffer, size_t count, std::vector <PWCollector_sample_t>& samples, int& buffer_offset, u64& num_samples)
{
    PWCollector_msg_t *msg;
    for (char *buff_ptr = &data_buffer[0]; count > 0; buff_ptr += (PW_MSG_HEADER_SIZE + msg->data_len), count -= (PW_MSG_HEADER_SIZE + msg->data_len)) {
        /*
         * There's no point in proceeding if we couldn't even read in the header!
         */
        if ((unsigned)count < PW_MSG_HEADER_SIZE) {
            memcpy(&data_buffer[0], buff_ptr, count);
            buffer_offset = (int)count;
            return PW_SUCCESS;
        }
        msg = (PWCollector_msg_t *)buff_ptr;
        unsigned msg_size = PW_MSG_HEADER_SIZE + msg->data_len;
        /*
         * We need a full message. If we don't have one then defer processing of this sample
         * until the next read.
         */
        if (count < msg_size) {
            memcpy(&data_buffer[0], buff_ptr, count);
            buffer_offset = (int)count;
            return PW_SUCCESS;
        }
        ++num_samples;
        // total_sample_size += msg_size;
        /*
         * OK, we have a complete message. Parse it.
         */
        u32 cpuidx = msg->cpuidx, data_type = msg->data_type, data_len = msg->data_len;
        db_fprintf(stderr, "OK: count = %lu, cpuidx = %lu, data_type = %lu, data_len = %lu\n", TO_UL(count), TO_UL(cpuidx), TO_UL(data_type), TO_UL(data_len));
        if (msg->data_type == C_STATE_MSR_SET) {
            pw_u64_vec_t& msr_set = m_per_thread_msr_sets[cpuidx];
            pw_u64_t *data_ptr = &msg->p_data;
            msr_set.insert(msr_set.begin(), data_ptr, data_ptr + MAX_MSR_ADDRESSES);
            db_copy(msr_set.begin(), msr_set.end(), std::ostream_iterator<pw_u64_t>(std::cerr, "\n"));
        } else if (msg->data_type == TSC_POSIX_MONO_SYNC) {
            tsc_posix_sync_msg_t *src = (tsc_posix_sync_msg_t *)&msg->p_data;
            assert(m_per_thread_clock_ratios[msg->cpuidx] == 0.0); // we should see one, and only one 'TSC_POSIX_MONO_SYNC' sample per logical CPU!
            m_per_thread_clock_ratios[msg->cpuidx] = (double)src->posix_mono_val / (double)src->tsc_val;
            db_fprintf(stderr, "[%d]: TSC = %llu CLK_GETTIME = %llu ratio = %.10f\n", msg->cpuidx, src->tsc_val, src->posix_mono_val, m_per_thread_clock_ratios[msg->cpuidx]);
        } else if (do_convert_msg_to_sample_i(msg, samples)) {
            fprintf(stderr, "ERROR converting message to samples!\n");
            return -PW_ERROR;
        }
    }
    return PW_SUCCESS;
};
/*
 * INTERNAL API:
 * Convert newer (i.e. >= v3.1.0) samples to older (i.e. v.3.0.X) format.
 *
 * @msg: the message to convert.
 * @samples: (reference to) a vector of converted samples.
 *
 * @returns 0 on success, -1 on error.
 */
// inline int pwr::WuParser::do_convert_msg_to_sample_i(PWCollector_msg_t *msg, std::list <PWCollector_sample_t>& samples)
int pwr::WuParser::do_convert_msg_to_sample_i(PWCollector_msg_t *msg, std::vector <PWCollector_sample_t>& samples)
{
    pw_u32_t cpuidx = msg->cpuidx, data_type = msg->data_type, data_len = msg->data_len;
    PWCollector_sample_t sample = {cpuidx, data_type, data_len, msg->tsc};
    std::vector <PWCollector_sample_t> dres_samples; // temp storage for d-res samples

    if (unlikely(m_busClockFreqKHz == 0)) {
        m_busClockFreqKHz = sysInfo.m_busClockFreq;
        m_busClockFreqMHz = sysInfo.m_busClockFreqMHz;
        if (m_busClockFreqMHz == 0.0) {
            m_busClockFreqMHz = (float)m_busClockFreqKHz / 1000.0;
        }
    }
    db_fprintf(stderr, "Converting: data_type = %u\n", data_type);

    switch (data_type) {
        case C_STATE:
            {
                pw_u64_vec_t& msr_set = m_per_thread_msr_sets[cpuidx];
                c_msg_t *src = (c_msg_t *)(&msg->p_data);
                c_sample_t *dst = &sample.c_sample;

                if (msr_set.empty()) {
                    fprintf(stderr, "OOB error!\n");
                    return PW_SUCCESS;
                }

                if (src->act_state > APERF) {
                    msr_set[src->act_state] = src->cx_msr_val;
                }
                dst->break_type = src->wakeup_type;
                dst->prev_state = src->req_state;
                dst->pid = src->wakeup_pid; dst->tid = src->wakeup_tid;
                dst->tps_epoch = src->tps_epoch;
                memcpy(dst->c_state_res_counts, &msr_set[0], sizeof(dst->c_state_res_counts));
                /*
                 * We store 'MPERF' values separately.
                 */
                RES_COUNT(*dst, MPERF) = src->mperf;
                /*
                 * We store the event 'TSC' in the LAST slot
                 */
                // RES_COUNT(*dst, C9) = src->wakeup_tsc;
                /*
                 * It is possible to get multiple TPS samples with the same CPU# and the same TSC.
                 * In this case, retain the LAST one.
                 * UPDATE: this is now auto-handled in 'pwr::WuParser::replay_trace_i()'
                 */
#if 0
                if (likely(samples.empty() == false)) {
                    if (samples.back().sample_type == C_STATE && samples.back().tsc == sample.tsc) {
                        db_fprintf(stderr, "Warning: prev sample tsc = %16llu, curr sample tsc = %16llu, DISCARDING PREVIOUS SAMPLE!\n", samples.back().tsc, msg->tsc);
                        if (g_do_debugging) {
                            std::cerr << samples.back() << sample;
                        }
                        samples.pop_back();
                    }
                }
#endif
                /*
                 * Also need to create an 'EVENT' sample corresponding to the wakeup type embedded in 'sample'.
                 * Do so now.
                 */
                {
                    PWCollector_sample_t event_sample = {cpuidx, FREE_SAMPLE, 0, src->wakeup_tsc};
                    sample_type_t event_type = FREE_SAMPLE;
                    pid_t event_pid = src->wakeup_pid, event_tid = src->wakeup_tid;
                    pw_u64_t event_val = src->wakeup_data;
                    pw_s32_t timer_init_cpu = src->timer_init_cpu; // only valid if 'src->wakeup_type' == PW_BREAK_TYPE_T!

                    switch (src->wakeup_type) {
                        case PW_BREAK_TYPE_I:
                            event_type = IRQ_SAMPLE;
                            event_sample.e_sample.data[0] = event_val;
                            event_sample.e_sample.data[1] = 0;
                            event_sample.e_sample.data[2] = 0;
                            event_sample.e_sample.data[3] = 0; /* MPERF value */
                            break;
                        case PW_BREAK_TYPE_T:
                            event_type = TIMER_SAMPLE;
                            if (m_is_from_axe && event_val > (unsigned int)NUM_ONLINE_CPUS()) {
                                /*
                                 * Convert the timer init timestamp (i.e. 'c_data') from a TSC-based value to a posix CLOCK_MONOTONIC-based value.
                                 * REQUIRED for TPSS compatibility!
                                 */
                                db_fprintf(stderr, "C-DATA before = %llu timer_init_cpu = %d\n", event_val, timer_init_cpu);
                                if (timer_init_cpu < 0 || timer_init_cpu >= NUM_ONLINE_CPUS()) { // sanities!
                                    db_fprintf(stderr, "WARNING: timer_init_cpu = %d, RESETTING TO msg->cpuidx (=%d)!\n", timer_init_cpu, msg->cpuidx);
                                    timer_init_cpu = msg->cpuidx;
                                }
                                event_val = (pw_u64_t)((double)event_val * m_per_thread_clock_ratios[timer_init_cpu]);
                                db_fprintf(stderr, "C-DATA after = %llu\n", event_val);
                            }
                            event_sample.e_sample.data[0] = event_pid;
                            event_sample.e_sample.data[1] = event_tid;
                            event_sample.e_sample.data[2] = event_val;
                            event_sample.e_sample.data[3] = 0; /* MPERF value */
                            break;
                        case PW_BREAK_TYPE_S:
                            /* Should NEVER happen! */
                            fprintf(stderr, "ERROR: sched_wakeup event?!\n");
                            assert(false);
                            break;
                        case PW_BREAK_TYPE_IPI:
                            event_type = IPI_SAMPLE;
                            event_sample.e_sample.data[0] = event_val;
                            event_sample.e_sample.data[1] = 0;
                            event_sample.e_sample.data[2] = 0;
                            event_sample.e_sample.data[3] = 0;
                            break;
                        case PW_BREAK_TYPE_W:
                            event_type = WORKQUEUE_SAMPLE;
                            event_sample.e_sample.data[0] = 0;
                            event_sample.e_sample.data[1] = 0;
                            event_sample.e_sample.data[2] = 0;
                            event_sample.e_sample.data[3] = 0;
                            break;
                        case PW_BREAK_TYPE_A:
                            db_fprintf(stderr, "WARNING: abort notification from driver?! Treating as unknown!\n");
                            // assert(false);
                        case PW_BREAK_TYPE_U: // fall-through
                            /*
                             * We need to discard this sample.
                             */
                            event_type = FREE_SAMPLE;
                            break;
                        default:
                            /* Should NEVER happen! */
                            fprintf(stderr, "ERROR: unknown break_type = %d\n", src->wakeup_type);
                            assert(false);
                            break;
                    }

                    if (likely(event_type != FREE_SAMPLE)) {
                        event_sample.sample_type = event_type;
                        samples.push_back(event_sample);
                    }
                }
            }
            break;
        case P_STATE:
            {
                p_msg_t *src = (p_msg_t *)&msg->p_data;
                p_sample_t *dst = &sample.p_sample;
#ifdef __arm__
                dst->frequency = src->perf_status_val * m_busClockFreqKHz;
                db_fprintf(stderr, "frequency = %u\n", dst->frequency);
#else
                pw_u32_t perf_status_val = src->perf_status_val;
                if (unlikely(m_perfStatusMask == 0)) {
                    pw_u16_t num_bits = (pw_u16_t)(sysInfo.m_perfBitsHigh - sysInfo.m_perfBitsLow + 1);
                    m_perfStatusMask = (1 << num_bits) - 1;
                    db_assert(true, "Calculated perf_status_mask = 0x%x\n", m_perfStatusMask);
                }
                assert(m_perfStatusMask);
                perf_status_val = (perf_status_val >> sysInfo.m_perfBitsLow) & m_perfStatusMask;
                // dst->frequency = perf_status_val * m_busClockFreqKHz;
                {
                    float __freq = (float)perf_status_val * m_busClockFreqMHz;
                    // dst->frequency = (int)(__freq + 0.5);
                    dst->frequency = GET_INT_FROM_FLOAT(__freq);
                    dst->frequency *= 1000;
                }
                // dst->frequency = perf_status_val * m_busClockFreqMHz * 1000;
                db_fprintf(stderr, "Perf-status val = %u, frequency = %u\n", perf_status_val, dst->frequency);
#endif
                dst->prev_req_frequency = src->prev_req_frequency;
                dst->is_boundary_sample = src->is_boundary_sample;
            }
            break;
        case K_CALL_STACK:
            {
                k_sample_t *src = (k_sample_t *)&msg->p_data;
                k_sample_t *dst = &sample.k_sample;
                *dst = *src;
            }
            break;
        case M_MAP:
            {
                sample.m_sample = *((m_sample_t *)&msg->p_data);
            }
            break;
        case IRQ_MAP:
            {
                i_sample_t *src = (i_sample_t *)&msg->p_data;
                i_sample_t *dst = &sample.i_sample;
                *dst = *src;
            }
            break;
        case PROC_MAP:
            {
                r_sample_t *src = (r_sample_t *)&msg->p_data;
                r_sample_t *dst = &sample.r_sample;
                *dst = *src;
            }
            break;
        case S_RESIDENCY:
            {
                s_residency_sample_t *src = (s_residency_sample_t *)&msg->p_data;
                s_residency_sample_t *dst = &sample.s_residency_sample;
                *dst = *src;
            }
            break;
        case D_RESIDENCY:
            {
                /*
                 * Need MULTIPLE samples for this!
                 */
                d_residency_msg_t *src = (d_residency_msg_t *)&msg->p_data;
                d_residency_sample_t *dst = NULL;
                int curr_dres_num = 0;
                for (int i=0; i<MAX_LSS_NUM_IN_SC; ++i) {
                    if (!curr_dres_num) {
                        dst = &sample.d_residency_sample;
                        dst->device_type = src->device_type;
                    }
                    if (src->mask & (pw_u64_t)(1 << i)) {
                        dst->mask[curr_dres_num] = i;
                        db_fprintf(stderr, "i = %d, curr_dres_num = %d\n", i, curr_dres_num);
                        memcpy(dst->d_residency_counters[curr_dres_num].data, src->d_residency_counters[i].data, sizeof(dst->d_residency_counters[curr_dres_num].data));
                        if (++curr_dres_num == PW_MAX_DEVICES_PER_SAMPLE) {
                            dst->num_sampled = curr_dres_num;
                            curr_dres_num = 0;
                            dres_samples.push_back(sample);
                        }
                    }
                }
                if (unlikely(curr_dres_num)) {
                    sample.d_residency_sample.num_sampled = curr_dres_num;
                    dres_samples.push_back(sample);
                }
            }
            break;
        case D_STATE:
            {
                d_state_sample_t *src = (d_state_sample_t *)&msg->p_data;
                d_state_sample_t *dst = &sample.d_state_sample;
                *dst = *src;
            }
            break;
        case SCHED_SAMPLE:
            {
                event_sample_t *src = (event_sample_t *)&msg->p_data;
                event_sample_t *dst = &sample.e_sample;
                *dst = *src;
            }
            break;
        case W_STATE:
            {
                w_wakelock_msg_t *src = (w_wakelock_msg_t *)&msg->p_data;
                w_sample_t *dst = &sample.w_sample;
                pw_u32_t cp_index = PW_STRIP_INITIAL_W_STATE_MAPPING_MASK(src->constant_pool_index);

                if (src->type != PW_WAKE_LOCK_INITIAL) {
                    db_fprintf(stderr, "W_STATE ORIGINAL cp_index = %u ", cp_index);
                    cp_index += m_maxProcWakelocksConstantPoolIndex + 1;
                    db_fprintf(stderr, "FINAL cp_index = %u\n", cp_index);
                } else {
                    db_fprintf(stderr, "Type = %u\n", src->type);
                }

                std::string wl_name = m_kernelWakelockConstantPool[cp_index];

                db_fprintf(stderr, "cp_index = %u, mapped name = %s\n", cp_index, wl_name.c_str());
                {
                    dst->type = (w_sample_type_t)src->type;
                    dst->tid = src->tid; dst->pid = src->pid;
                    dst->expires = src->expires;
                    STRNCPY(dst->name, wl_name.c_str(), sizeof(dst->name)-1);
                    dst->name[sizeof(dst->name)-1] = '\0';
                    memcpy(dst->proc_name, src->proc_name, PW_MAX_PROC_NAME_SIZE);
                }
                if (g_do_debugging) {
                    std::cerr << "Converted W_STATE sample = " << sample;
                }
                if (unlikely(wl_name.length() == 0)) {
                    /*
                     * We couldn't find a constant-pool entry message for this index. This is caused
                     * because the power driver sent us the constant-pool message out-of-order. We
                     * handle this situation by keeping track of these "incomplete" wakelock messages
                     * and then patching them up later (when we encounter the constant pool messages).
                     */
                    m_incompleteWakelockMessages[cp_index].push_back(sample);
                    /*
                     * We can't enqueue 'sample' here (we must wait for the corresponding constant-pool
                     * message).
                     */
                    if (g_do_debugging) {
                        std::cerr << "WARNING: deferring processing of wakelock sample: " << sample;
                    }
                    return PW_SUCCESS;
                }
            }
            break;
        case DEV_MAP:
            {
                dev_sample_t *src = (dev_sample_t *)&msg->p_data;
                dev_sample_t *dst = &sample.dev_sample;
                *dst = *src;
            }
            break;
        case U_STATE:
            {
                u_wakelock_msg_t *src = (u_wakelock_msg_t *)&msg->p_data;
                u_sample_t *dst = &sample.u_sample;
                pw_u32_t cp_index = src->constant_pool_index;
                std::string wl_tag = m_userWakelockConstantPool[cp_index];
                size_t len = wl_tag.length();

                if (g_do_debugging) {
                    std::cerr << "cp_index = " << cp_index << ", mapped tag = " << wl_tag << "\n";
                }
                {
                    dst->type = (u_sample_type_t)src->type;
                    dst->flag = (u_sample_flag_t)src->flag;
                    dst->pid = src->pid;
                    dst->uid = src->uid;
                    dst->count = src->count;
                    assert(len <= PW_MAX_WAKELOCK_NAME_SIZE); // sanity!
                    memcpy(dst->tag, wl_tag.c_str(), len);
                }
                if (g_do_debugging) {
                    std::cerr << "Converted U_STATE sample = " << sample;
                }
            }
            break;
        case PKG_MAP:
            {
                pkg_sample_t *src = (pkg_sample_t *)&msg->p_data;
                pkg_sample_t *dst = &sample.pkg_sample;
                *dst = *src;
            }
            break;
        case CONSTANT_POOL_ENTRY:
            {
                constant_pool_msg_t *src = (constant_pool_msg_t *)&msg->p_data;
                pw_u32_t cp_index = src->entry_index;

                db_fprintf(stderr, "constant_pool_msg: cpu = %d, tsc = %llu, type = %u, index = %u, entry = %s\n", msg->cpuidx, msg->tsc, src->entry_type, cp_index, src->entry);

                sample.sample_type = src->entry_type;
                if (sample.sample_type == W_STATE) {
                    /*
                     * Kernel wakelock mapping.
                     */
                    if (PW_HAS_INITIAL_W_STATE_MAPPING_MASK(cp_index)) {
                        cp_index = PW_STRIP_INITIAL_W_STATE_MAPPING_MASK(cp_index); // NOP if not PW_INITIAL_WAKE_LOCK constant-pool index
                        m_maxProcWakelocksConstantPoolIndex = std::max(m_maxProcWakelocksConstantPoolIndex, (pw_s32_t)cp_index);
                        db_fprintf(stderr, "INITIAL wakelock! cp_index = %d, max index = %u\n", cp_index, m_maxProcWakelocksConstantPoolIndex);
                    } else {
                        db_fprintf(stderr, "COLLECTION wakelock!\n");
                        /*
                         * Kernel wakelocks initialized DURING the collection are shifted by a factor of 'm_maxProcWakelocksConstantPoolIndex + 1'.
                         * This allows us to service ALL kernel wakelocks (i.e. those initialized during the collection and those obtained
                         * from the "/proc/wakelocks" file before the collection starts) from the same constant pool
                         */
                        db_fprintf(stderr, "ORIGINAL cp_index = %u", cp_index);
                        cp_index += m_maxProcWakelocksConstantPoolIndex + 1;
                        db_fprintf(stderr, "FINAL cp_index = %u\n", cp_index);
                    }
                    m_kernelWakelockConstantPool[cp_index] = src->entry;
                } else { 
                    /*
                     * Userspace wakelock mapping.
                     */
                    m_userWakelockConstantPool[cp_index] = src->entry;
                }
                /*
                 * Did we defer processing of any 'w-state' samples because we hadn't seen the corresponding constant-pool message yet?
                 */
                if (unlikely(m_incompleteWakelockMessages.find(cp_index) != m_incompleteWakelockMessages.end())) {
                    if (g_do_debugging) {
                        std::cerr << "FOUND deferred cp_index = " << cp_index << ", name = " << src->entry << "\n";
                    }
                    std::vector <PWCollector_sample_t>& deferredMessages = m_incompleteWakelockMessages[cp_index];
                    for (std::vector<PWCollector_sample_t>::iterator iter = deferredMessages.begin(); iter != deferredMessages.end(); ++iter) {
                        STRNCPY(iter->w_sample.name, src->entry, sizeof(iter->w_sample.name)-1);
                    }
                    if (g_do_debugging) {
                        std::copy(deferredMessages.begin(), deferredMessages.end(), std::ostream_iterator<PWCollector_sample_t>(std::cerr, ""));
                    }
                    samples.insert(samples.end(), deferredMessages.begin(), deferredMessages.end());
                    deferredMessages.clear();
                }
            }
            /*
             * No further action required.
             */
            return PW_SUCCESS;
            break;
        case CPUHOTPLUG_SAMPLE:
            {
                event_sample_t *src = (event_sample_t *)&msg->p_data;
                event_sample_t *dst = &sample.e_sample;
                *dst = *src;
            }
            break;
        default:
            fprintf(stderr, "Detected invalid Msg type = %u. Corrupted input file?!\n", data_type);
            return -PW_ERROR;
    }
    if (sample.sample_type == FREE_SAMPLE) {
        assert(false);
    }
    if (dres_samples.empty() == false) {
        samples.insert(samples.end(), dres_samples.begin(), dres_samples.end());
    } else {
        samples.push_back(sample);
    }
    return PW_SUCCESS;
};

/*
 * EXTERNAL API:
 * Set the input directory and file names.
 *
 * @dir: the directory name.
 * @file: the actual file name.
 */
void pwr::WuParser::set_wuwatch_output_file_name(const std::string& file_path)
{
    assert(file_path.size());
    m_combined_input_file_name = file_path;
    db_fprintf(stderr, "WUDUMP has input file = %s\n", m_combined_input_file_name.c_str());
};

/*
 * EXTERNAL API:
 * Main function:
 * (1) Read samples.
 * (2) Parse them, exctracting relevent information.
 * (3) Print out results.
 *
 * @returns: 0 on success, -1 on error.
 */
int pwr::WuParser::do_work(void)
{
    if (do_read_i()) {
        fprintf(stderr, "ERROR reading input data!\n");
        return -PW_ERROR;
    }

    /*
     * For Windows Wuwatch output, no postprocessing is required.
     */
    if (sysInfo.m_osName.find("Window") != std::string::npos) {
	fprintf(stderr, "OSName = %s \n", sysInfo.m_osName.c_str());    
        return PW_SUCCESS;
    }

    if (do_parse_i()) {
        fprintf(stderr, "ERROR parsing data!\n");
        return -PW_ERROR;
    }

    if (do_finalize_and_collate_i()) {
        fprintf(stderr, "ERROR soring and collating samples!\n");
        return -PW_ERROR;
    }

    return PW_SUCCESS;
};

int pwr::WuParser::do_work(sample_list_t& samples)
{
    /*
     * Make sure we copy over relevant information from
     * the 'SystemInfo' class! We only need to do this
     * in the case where we haven't parsed a 'system param'
     * file (e.g. when we're called from the AXE power collector).
     */
    {
        m_availableFrequenciesKHz.insert(m_availableFrequenciesKHz.begin(), sysInfo.m_availableFrequenciesKHz.begin(), sysInfo.m_availableFrequenciesKHz.end());
    }

    m_origSamples.insert(m_origSamples.end(), samples.begin(), samples.end());

    if (do_parse_i()) {
        fprintf(stderr, "ERROR parsing data!\n");
        return -PW_ERROR;
    }

    if (do_finalize_and_collate_i()) {
        fprintf(stderr, "ERROR soring and collating samples!\n");
        return -PW_ERROR;
    }

    return PW_SUCCESS;
};

/*
 * PRIVATE API
 * Default constructor -- privatized for Singleton-compatibility.
 */
pwr::WuData::WuData() : m_init_complete(false) {};
/*
 * PRIVATE API
 * Default destructor -- privatized for Singleton-compatibility.
 */
pwr::WuData::~WuData() {
};

/*
 * PUBLIC API
 * Return a (globally persistent) WuData instance.
 */
pwr::WuData *pwr::WuData::instance()
{
    if (!s_data) {
        s_data = new WuData;
    }
    return s_data;
};

/*
 * PUBLIC API
 * Destroy the (globally persistent) WuData instance.
 */
void pwr::WuData::destroy()
{
    delete s_data;
    s_data = NULL;
};


/*
 * PUBLIC API
 * Entry point to reading and parsing process.
 *
 * @file_path: path to the wuwatch output file.
 * @should_calc_c1: should we calculate C1 residencies?
 * @is_from_axe: is this being used to import data into AXE? <DEFAULT = false>
 * @should_dump_orig_samples: should we dump the (unprocessed) samples? <DEFAULT = false>
 * @should_dump_sample_stats: should we dump some statistics? <DEFAULT = false>
 *
 * @returns:
 *          0 ==>   OK
 *          -1 ==>  Failure.
 */
int pwr::WuData::do_read_and_process(const std::string& file_path, bool should_calc_c1, bool is_from_axe, bool should_dump_orig_samples, bool should_dump_sample_stats)
{
    /*
     * Determine if user wants us to dump unprocessed samples.
     */
    if (should_dump_orig_samples == false) {
        should_dump_orig_samples = CheckEnv("PW_DO_DUMP_ORIG_SAMPLES");
    }
    /*
     * Determine if we should convert S,D residencies from 'usecs' to TSC ticks.
     * We do this conversion if:
     * 1. The user FORCES us to do so OR
     * 2. We're called from AXE AND
     * 3. We're importing data from a 3.0.1 or newer driver.
     * Note that (3) CANNOT be checked at this stage -- it must wait until we've parsed
     * the 'sys_params_found' section of 'wuwatch_output.ww1'.
     */
    bool force_s_d_res_to_tsc = false;
    {
        force_s_d_res_to_tsc = CheckEnv("PW_DO_FORCE_S_D_RES_TO_TSC");
        if (force_s_d_res_to_tsc) { 
            db_fprintf(stderr, "Warning: forcing S,D residency to TSC ticks!\n");
        }
    }
    if (is_from_axe) {
        db_fprintf(stderr, "CALLED FROM AXE IMPORT!\n");
    }
    WuParser *wuparserObj = new WuParser(sysInfo, should_calc_c1, should_dump_orig_samples, should_dump_sample_stats, force_s_d_res_to_tsc, is_from_axe);
    wuparserObj->set_wuwatch_output_file_name(file_path);
    /*
     * 'WuParser' accesses macros that require 'm_init_complete' to
     * be set.
     */
    m_init_complete = true;

    if (wuparserObj->do_work()) {
        /*
         * Something went wrong. Free up memory, reset the
         * 'init' flag and die.
         */
        m_init_complete = false;
        delete wuparserObj;
        return -PW_ERROR;
    }
    /*
     * OK, copy samples and trace maps etc. over.
     */
    if (sysInfo.m_osName.find("Window") != std::string::npos) {
        m_samples = sample_vec_t(wuparserObj->m_origSamples.size());
        std::copy(wuparserObj->m_origSamples.begin(), wuparserObj->m_origSamples.end(), m_samples.begin());
    }
    else
    {
        m_samples = sample_vec_t(wuparserObj->m_output_samples.size());
        std::copy(wuparserObj->m_output_samples.begin(), wuparserObj->m_output_samples.end(), m_samples.begin());
        /*
        m_trace_pair_map = wuparserObj->m_trace_pair_map;
        m_trace_vec = wuparserObj->m_trace_vec;
        */
    }

    delete wuparserObj;
    return PW_SUCCESS;
};

/*
 * PUBLIC API
 * Entry point to parsing previously read samples. Used primarily by AXE.
 *
 */
int pwr::WuData::do_process(sample_list_t& samples, SystemInfo& systemInfo)
{
    sysInfo = systemInfo;
    /*
     * Determine if user wants us to dump unprocessed samples.
     */
    bool should_dump_orig = CheckEnv("PW_DO_DUMP_ORIG_SAMPLES");
    /*
     * Determine if useer wants us to dump sample stats.
     */
    bool should_dump_sample_stats = CheckEnv("PW_DO_DUMP_SAMPLE_STATS");
    /*
     * Determine if we should convert S,D residencies from 'usecs' to TSC ticks.
     * We do this conversion if:
     * 1. The user FORCES us to do so OR
     * 2. We're called from AXE AND
     * 3. We're importing data from a 3.0.1 or newer driver.
     */
    bool should_convert_s_d = WUWATCH_VERSION(sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther) >= WUWATCH_VERSION(3, 0, 1);
    db_fprintf(stderr, "driver major = %d, minor = %d, other = %d, version = %u\n", sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther, WUWATCH_VERSION(sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther)); 
    if (!should_convert_s_d) {
        bool should_force = CheckEnv("PW_DO_FORCE_S_D_RES_TO_TSC");
        if (should_force) {
            should_convert_s_d = true;
            fprintf(stderr, "Warning: forcing S,D residency to TSC ticks!\n");
        }
    }
    /*
     * Construct the parser object.
     */
    WuParser *wuparserObj = new WuParser(sysInfo, true /*should_calc_c1*/, should_dump_orig, should_dump_sample_stats, should_convert_s_d /* should convert {S,D} usecs to TSC */, true /* is from AXE */);
    /*
     * 'WuParser' accesses macros that require 'm_init_complete' to
     * be set.
     */
    m_init_complete = true;

    if (wuparserObj->do_work(samples)) {
        /*
         * Something went wrong. Free up memory, reset the
         * 'init' flag and die.
         */
        m_init_complete = false;
        delete wuparserObj;
        return -PW_ERROR;
    }
    /*
     * OK, copy samples and trace maps etc. over.
     */
    {
        m_samples = sample_vec_t(wuparserObj->m_output_samples.size());
        std::copy(wuparserObj->m_output_samples.begin(), wuparserObj->m_output_samples.end(), m_samples.begin());
    }

    delete wuparserObj;
    return PW_SUCCESS;
};

int pwr::WuData::do_process(const std::vector<char>& data, SystemInfo& systemInfo)
{
    sysInfo = systemInfo;
    /*
     * Determine if user wants us to dump unprocessed samples.
     */
    bool should_dump_orig = CheckEnv("PW_DO_DUMP_ORIG_SAMPLES");
    /*
     * Determine if useer wants us to dump sample stats.
     */
    bool should_dump_sample_stats = CheckEnv("PW_DO_DUMP_SAMPLE_STATS");
    /*
     * Determine if we should convert S,D residencies from 'usecs' to TSC ticks.
     * We do this conversion if:
     * 1. The user FORCES us to do so OR
     * 2. We're called from AXE AND
     * 3. We're importing data from a 3.1 or newer driver.
     */
    bool should_convert_s_d = WUWATCH_VERSION(sysInfo.m_driverMajor, sysInfo.m_driverMinor, sysInfo.m_driverOther) >= WUWATCH_VERSION(3, 0, 1);
    if (!should_convert_s_d) {
        bool should_force = CheckEnv("PW_DO_FORCE_S_D_RES_TO_TSC");
        if (should_force) {
            should_convert_s_d = true;
            fprintf(stderr, "Warning: forcing S,D residency to TSC ticks!\n");
        }
    }
    /*
     * Construct the parser object.
     */
    WuParser *wuparserObj = new WuParser(sysInfo, true /*should_calc_c1*/, should_dump_orig, should_dump_sample_stats, should_convert_s_d /* should convert {S,D} usecs to TSC */, true /* is from AXE */);
    /*
     * 'WuParser' accesses macros that require 'm_init_complete' to
     * be set.
     */
    m_init_complete = true;

    /*
     * Post v310, power drivers return variable-length messages. Translate them.
     */
    std::vector<char>& tmp_buffer = const_cast<std::vector<char>&>(data);
    std::vector <PWCollector_sample_t> samples;
    int buffer_offset = 0; // nop
    u64 num_samples = 0; //, total_sample_size = 0;

    if (wuparserObj->do_parse_messages_i(tmp_buffer, (size_t)tmp_buffer.size(), samples, buffer_offset, num_samples)) {
        db_fprintf(stderr, "ERROR parsing messages\n");
        m_init_complete = false;
        delete wuparserObj;
        return -PW_ERROR;
    }
    std::list <PWCollector_sample_t> tmp_samples;
    tmp_samples.insert(tmp_samples.end(), samples.begin(), samples.end());
    if (wuparserObj->do_work(tmp_samples)) {
        /*
         * Something went wrong. Free up memory, reset the
         * 'init' flag and die.
         */
        m_init_complete = false;
        delete wuparserObj;
        return -PW_ERROR;
    }
    /*
     * OK, copy samples and trace maps etc. over.
     */
    {
        m_samples = sample_vec_t(wuparserObj->m_output_samples.size());
        std::copy(wuparserObj->m_output_samples.begin(), wuparserObj->m_output_samples.end(), m_samples.begin());
    }

    delete wuparserObj;
    return PW_SUCCESS;
};
/*
 * PUBLIC API
 * Retrieve system configuration information.
 *
 * @returns: the system configuration information.
 */
const pwr::SystemInfo& pwr::WuData::getSystemInfo() const
{
    assert(m_init_complete);
    return sysInfo;
};

/*
 * PUBLIC API
 * Retrieve list of PWCollector samples returned by the driver.
 *
 * @returns: the list of (parsed) PWCollector samples
 */
const sample_vec_t& pwr::WuData::getSamples() const
{
    assert(m_init_complete);
    return m_samples;
};

/*
 * PUBLIC API
 * Retrieve TID <--> Call trace mappings returned by the driver
 * and also mappings generated by the hook library.
 *
 * @returns: the call trace mapping.
 */
/*
const trace_pair_map_t& pwr::WuData::get_tid_backtrace_map() const
{
    assert(m_init_complete);
    return m_trace_pair_map;
};
*/
/*
 * EXTERNAL API:
 * Helper function to pretty-print an individual
 * power sample.
 */
/*
void operator<<(std::ostream& os, const PWCollector_sample_t& sample)
{
    os << sample;
};
*/

/*
 * EXTERNAL API:
 * Helper function to pretty-print an individual
 * power sample.
 */
std::ostream& operator<<(std::ostream& os, const PWCollector_sample_t& sample)
{
    const c_sample_t *cs;
    const r_sample_t *rs;
    const i_sample_t *is;
    const w_sample_t *ws;
    int cpu;
    unsigned long long tsc, mperf, c1, c2, c3, c4, c5, c6, c9;

    cpu = sample.cpuidx;
    tsc = sample.tsc;

    os << cpu << "\t" << sample.tsc << "\t" << s_long_sample_names[sample.sample_type];

    if (sample.sample_type == C_STATE) {
        cs = &sample.c_sample;
        mperf = RES_COUNT(*cs, MPERF);
        c1 = RES_COUNT(*cs, APERF);
        c2 = RES_COUNT(*cs, C2);
        c3 = RES_COUNT(*cs, C3);
        c4 = RES_COUNT(*cs, C4);
        c5 = RES_COUNT(*cs, C5);
        c6 = RES_COUNT(*cs, C6);

        c9 = RES_COUNT(*cs, C9);

        if (false) {
            fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
            fprintf(stderr, "\t%llu\t%llu", mperf, c1);
        }

        os << "\t" << mperf << "\t" << c1;

        if (PW_IS_SALTWELL(pwr::WuData::instance()->getSystemInfo().m_cpuModel)) {
            os << "\t" << c2 << "\t" << c4 << "\t" << c5 << "\t" << c6;
            if (false) {
                fprintf(stderr, "\t%llu\t%llu\t%llu\t%llu", c2, c4, c5, c6);
            }
        } else {
            os << "\t" << c3 << "\t" << c6;
        }
        os << "\t" << sample.c_sample.prev_state << "\t" << sample.c_sample.tps_epoch << "\t" << sample.c_sample.break_type << "\t" << sample.c_sample.c_data;
        if (false) {
            // HACK!
            fprintf(stderr, "\t%u\t%u\t%llu\n", sample.c_sample.prev_state, sample.c_sample.tps_epoch, sample.c_sample.c_data);
        }
    }
    else if (sample.sample_type == P_STATE) {
        os << "\t" << sample.p_sample.prev_req_frequency << "\t" << sample.p_sample.frequency << "\t" << GET_BOOL_STRING(sample.p_sample.is_boundary_sample);
        if (true) {
            os << "\t" << (pw_u32_t)(sample.p_sample.unhalted_core_value >> 32) << "\t" << (pw_u32_t)(sample.p_sample.unhalted_core_value & 0xffffffff) << "\t" << sample.p_sample.unhalted_ref_value;
        }
    }
    else if (sample.sample_type == IRQ_MAP) {
        is = &sample.i_sample;
        os << "\t" << is->irq_num << "\t" << is->irq_name;
        if (false) {
            fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
            fprintf(stderr, "\t%d\t%s\n", is->irq_num, is->irq_name);
        }
    }
    else if (sample.sample_type == PROC_MAP) {
        rs = &sample.r_sample;
        os << "\t" << rs->type << "\t" << rs->tid << "\t" << rs->pid << "\t" << rs->proc_name;
    }
    else if (sample.sample_type == IRQ_SAMPLE) {
        os << "\t" << sample.e_sample.data[0];
        if (false) {
            fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
            fprintf(stderr, "\t%llu\n", sample.e_sample.data[0]);
        }
    }
    else if (sample.sample_type == TIMER_SAMPLE) {
        os << "\t" << sample.e_sample.data[0] << "\t" << sample.e_sample.data[1] << "\t" << sample.e_sample.data[2];
        if (false) {
            fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
            fprintf(stderr, "\t%llu\t%llu\t%llu\n", sample.e_sample.data[0], sample.e_sample.data[1], sample.e_sample.data[2]);
        }
    }
    else if (sample.sample_type == SCHED_SAMPLE) {
        os << "\t" << sample.e_sample.data[0] << "\t" << sample.e_sample.data[1] << "\t" << sample.e_sample.data[2];
        if (false) {
            fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
            fprintf(stderr, "\t%llu\t%llu\t%llu\n", sample.e_sample.data[0], sample.e_sample.data[1], sample.e_sample.data[2]);
        }
    }
    else if (false && sample.sample_type == IPI_SAMPLE) {
        fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
        fprintf(stderr, "\n");
    }
    else if (sample.sample_type == DEV_MAP) {
        const std::string& dev_type = sample.dev_sample.dev_type == PW_NORTH_COMPLEX ? "NC" : "SC";
        os << "\t" << dev_type << "\t" << sample.dev_sample.dev_num << "\t" << sample.dev_sample.dev_short_name << "\t" << sample.dev_sample.dev_long_name;
    } else if (sample.sample_type == W_STATE) {
        ws = &sample.w_sample;
        std::string w_type;
        switch (ws->type) {
            case PW_WAKE_LOCK:
                w_type = "LOCK";
                break;
             case PW_WAKE_LOCK_INITIAL:
                w_type = "LOCK_INIT";
                break;
             case PW_WAKE_LOCK_TIMEOUT:
                w_type = "LOCK_TIMEOUT";
                break;
             case PW_WAKE_UNLOCK:
                w_type = "UNLOCK";
                break;
             case PW_WAKE_UNLOCK_ALL:
                w_type = "UNLOCKALL";
                break;
        }
        os << "\t" << w_type << "\t" << ws->name << "\t" << ws->proc_name << "\t" << ws->pid << "\t" << ws->tid;
    }
    else if (sample.sample_type == CPUHOTPLUG_SAMPLE) {
        os << "\t" << sample.e_sample.data[0] << "\t" << sample.e_sample.data[1] << "\t" << sample.e_sample.data[2];
        if (false) {
            fprintf(stderr, "%d\t%llu\t%s", cpu, sample.tsc, s_long_sample_names[sample.sample_type]);
            fprintf(stderr, "\t%llu\t%llu\t%llu\n", sample.e_sample.data[0], sample.e_sample.data[1], sample.e_sample.data[2]);
        }
    }
    else if (sample.sample_type == S_RESIDENCY) {
        for (int i=0; i<6; ++i) {
            os << "\t" << sample.s_residency_sample.data[i];
        }
    }
    else if (sample.sample_type == D_RESIDENCY) {
        const u16 *mask = sample.d_residency_sample.mask;
        int num_sampled = sample.d_residency_sample.num_sampled;
        for (int i = 0; i < num_sampled; ++i) { // from 0 --> 2
            pw_u32_t id = mask[i];
            if (id < max_lss_num_in_sc) {
                os << "\t" << id;
                for (int j=0; j<4; ++j) {
                    os << "\t" << sample.d_residency_sample.d_residency_counters[i].data[j];
                }
            }
        }
    }
    else if (sample.sample_type == D_STATE) {
        const u32 *states = sample.d_state_sample.states;
        const char type = sample.d_state_sample.device_type;
        os << "\t" << (int)type << "\t" << states[0] << "\t" << states[1];
    }
    os << std::endl;
    os.flush();
    return os;
};

/*
 * EXTERNAL API:
 * Helper function to pretty-print a pair.
 */
void operator<<(std::ostream& os, const int_pair_t& pair)
{
    os << pair.first << "\t" << pair.second << std::endl;
};
/*
 * EXTERNAL API:
 * Helper function to compare two c-state samples.
 *
 * @s1, @s2: the two samples to compare.
 *
 * @returns: "true" ==> the two samples are 'EQUAL', "false" otherwise.
 * Equality is defined by comparing the MSR-sets of the two samples.
 */
bool operator==(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2)
{
    const c_sample_t& cs1 = s1.c_sample, cs2 = s2.c_sample;
    /*
     * Unlike the case when we need to know WHICH C-state MSR counter, 
     * iteration order doesn't matter in this case: we only
     * need to know if ANY C-state MSR counted.
     */
    for (int i=C2; i<MAX_MSR_ADDRESSES; ++i) {
        if (RES_COUNT(cs1, i) != RES_COUNT(cs2, i)) {
            return false;
        }
    }
    return true;
};
/*
 * EXTERNAL API:
 * Helper function to compare two c-state samples.
 */
bool operator!=(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2)
{
    return !(s1 == s2);
};

/*
 * EXTERNAL API:
 * Helper function used to sort (in ascending TSC order) the various PWCollector
 * samples returned by the driver.
 *
 * @s1, @s2: the two samples to compare.
 *
 * @returns: "true" ==> s1.tsc < s2.tsc, "false" otherwise.
 */
bool operator<(const PWCollector_sample_t& s1, const PWCollector_sample_t& s2)
{
    return s1.tsc < s2.tsc;
};
