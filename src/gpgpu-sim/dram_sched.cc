// Copyright (c) 2009-2011, Tor M. Aamodt, Ali Bakhoda, George L. Yuan,
// The University of British Columbia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution. Neither the name of
// The University of British Columbia nor the names of its contributors may be
// used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "dram_sched.h"
#include "../abstract_hardware_model.h"
#include "gpu-misc.h"
#include "gpu-sim.h"
#include "mem_latency_stat.h"

frfcfs_scheduler::frfcfs_scheduler(const memory_config *config, dram_t *dm,
                                   memory_stats_t *stats) {
  m_config = config; // bank개수 등등 memory config내용
  m_stats = stats; // 
  m_num_pending = 0; // buffer에 pending하고 있는 request의 개수
  m_num_write_pending = 0; 
  m_dram = dm;
  m_queue = new std::list<dram_req_t *>[m_config->nbk]; // 단순히 request를 순차적으로 저장하는 queue.
  m_bins = new std::map<
      unsigned, std::list<std::list<dram_req_t *>::iterator> >[m_config->nbk]; // Map(key, value), key = row, value = req list
  m_last_row =
      new std::list<std::list<dram_req_t *>::iterator> *[m_config->nbk]; // bins에서 선택한 value (= req list)
  curr_row_service_time = new unsigned[m_config->nbk]; // Bank별로 현재 row가 service되고 있는 time
  row_service_timestamp = new unsigned[m_config->nbk]; // ?
  for (unsigned i = 0; i < m_config->nbk; i++) {
    m_queue[i].clear();
    m_bins[i].clear();
    m_last_row[i] = NULL;
    curr_row_service_time[i] = 0;
    row_service_timestamp[i] = 0;
  }
  if (m_config->seperate_write_queue_enabled) { // request queue를 read/write로 세분화하여 구분한다.
    m_write_queue = new std::list<dram_req_t *>[m_config->nbk];
    m_write_bins = new std::map<
        unsigned, std::list<std::list<dram_req_t *>::iterator> >[m_config->nbk];
    m_last_write_row =
        new std::list<std::list<dram_req_t *>::iterator> *[m_config->nbk];

    for (unsigned i = 0; i < m_config->nbk; i++) {
      m_write_queue[i].clear();
      m_write_bins[i].clear();
      m_last_write_row[i] = NULL;
    }
  }
  m_mode = READ_MODE;
}

void frfcfs_scheduler::add_req(dram_req_t *req) {
  if (m_config->seperate_write_queue_enabled && req->data->is_write()) { //write queue가 구분되어 있고 현재 write req가 들어왔을 때
    assert(m_num_write_pending < m_config->gpgpu_frfcfs_dram_write_queue_size);
    m_num_write_pending++;
    m_write_queue[req->bk].push_front(req);
    std::list<dram_req_t *>::iterator ptr = m_write_queue[req->bk].begin();
    // Open Page를 위해 같은 row 끼리 req를 모으는 형식
    m_write_bins[req->bk][req->row].push_front(ptr);  // newest reqs to the
                                                      // front
  } else { //위와 동일
    assert(m_num_pending < m_config->gpgpu_frfcfs_dram_sched_queue_size);
    m_num_pending++;
    m_queue[req->bk].push_front(req);
    std::list<dram_req_t *>::iterator ptr = m_queue[req->bk].begin();
    m_bins[req->bk][req->row].push_front(ptr);  // newest reqs to the front
  }
}

void frfcfs_scheduler::data_collection(unsigned int bank) {
  if (m_dram->m_gpu->gpu_sim_cycle > row_service_timestamp[bank]) {
    curr_row_service_time[bank] =
        m_dram->m_gpu->gpu_sim_cycle - row_service_timestamp[bank];
    if (curr_row_service_time[bank] >
        m_stats->max_servicetime2samerow[m_dram->id][bank])
      m_stats->max_servicetime2samerow[m_dram->id][bank] =
          curr_row_service_time[bank];
  }
  curr_row_service_time[bank] = 0;
  row_service_timestamp[bank] = m_dram->m_gpu->gpu_sim_cycle;
  if (m_stats->concurrent_row_access[m_dram->id][bank] >
      m_stats->max_conc_access2samerow[m_dram->id][bank]) {
    m_stats->max_conc_access2samerow[m_dram->id][bank] =
        m_stats->concurrent_row_access[m_dram->id][bank];
  }
  m_stats->concurrent_row_access[m_dram->id][bank] = 0;
  m_stats->num_activates[m_dram->id][bank]++;
}

dram_req_t *frfcfs_scheduler::schedule(unsigned bank, unsigned curr_row) {
  // row
  bool rowhit = true;
  std::list<dram_req_t *> *m_current_queue = m_queue;
  std::map<unsigned, std::list<std::list<dram_req_t *>::iterator> >
      *m_current_bins = m_bins;
  std::list<std::list<dram_req_t *>::iterator> **m_current_last_row =
      m_last_row;

  // Threshold를 기준으로 read/write중 어떤 mode를 선택할 건지 결정
  if (m_config->seperate_write_queue_enabled) {
    if (m_mode == READ_MODE &&
        ((m_num_write_pending >= m_config->write_high_watermark)
         // || (m_queue[bank].empty() && !m_write_queue[bank].empty())
         )) {
      m_mode = WRITE_MODE;
    } else if (m_mode == WRITE_MODE &&
               ((m_num_write_pending < m_config->write_low_watermark)
                //  || (!m_queue[bank].empty() && m_write_queue[bank].empty())
                )) {
      m_mode = READ_MODE;
    }
  }

  if (m_mode == WRITE_MODE) {
    m_current_queue = m_write_queue;
    m_current_bins = m_write_bins;
    m_current_last_row = m_last_write_row;
  }

  if (m_current_last_row[bank] == NULL) { // 현재 service되고 있는 row가 없다!
    if (m_current_queue[bank].empty()) return NULL;

    std::map<unsigned, std::list<std::list<dram_req_t *>::iterator> >::iterator
        bin_ptr = m_current_bins[bank].find(curr_row);
    if (bin_ptr == m_current_bins[bank].end()) { //현재 serive되고 있던 row와 동일한 row를 가진 req_list가 없다. 따라서 request_queue에서 FIFO 정책으로 하나를 가져오자.
      dram_req_t *req = m_current_queue[bank].back(); // FIFO 정책에 따라 request queue중에 가장 오래된 것을 다음 req로 설정
      bin_ptr = m_current_bins[bank].find(req->row);
      assert(bin_ptr !=
             m_current_bins[bank].end());  // where did the request go???
      m_current_last_row[bank] = &(bin_ptr->second);
      data_collection(bank);
      rowhit = false;
    } else {
      m_current_last_row[bank] = &(bin_ptr->second);
      rowhit = true;
    }
  }
  std::list<dram_req_t *>::iterator next = m_current_last_row[bank]->back(); //현재 req_list에서 하나 가져옴
  dram_req_t *req = (*next);

  // rowblp stats
  m_dram->access_num++;
  bool is_write = req->data->is_write();
  if (is_write)
    m_dram->write_num++;
  else
    m_dram->read_num++;

  if (rowhit) {
    m_dram->hits_num++;
    if (is_write)
      m_dram->hits_write_num++;
    else
      m_dram->hits_read_num++;
  }

  m_stats->concurrent_row_access[m_dram->id][bank]++;
  m_stats->row_access[m_dram->id][bank]++;
  m_current_last_row[bank]->pop_back(); // 여기서 pop

  m_current_queue[bank].erase(next); //여기서 request queue에 있는 동일한 req도 삭제
  if (m_current_last_row[bank]->empty()) {
    m_current_bins[bank].erase(req->row);
    m_current_last_row[bank] = NULL;
  }
#ifdef DEBUG_FAST_IDEAL_SCHED
  if (req)
    printf("%08u : DRAM(%u) scheduling memory request to bank=%u, row=%u\n",
           (unsigned)gpu_sim_cycle, m_dram->id, req->bk, req->row);
#endif

  if (m_config->seperate_write_queue_enabled && req->data->is_write()) {
    assert(req != NULL && m_num_write_pending != 0);
    m_num_write_pending--;
  } else {
    assert(req != NULL && m_num_pending != 0);
    m_num_pending--;
  }

  return req;
}

void frfcfs_scheduler::print(FILE *fp) {
  for (unsigned b = 0; b < m_config->nbk; b++) {
    printf(" %u: queue length = %u\n", b, (unsigned)m_queue[b].size());
  }
}

void dram_t::scheduler_frfcfs() {
  unsigned mrq_latency;
  frfcfs_scheduler *sched = m_frfcfs_scheduler;
  // mrqq : L2 miss난 애들이 FIFO에 저장되어 있는 것들
  while (!mrqq->empty()) {
    dram_req_t *req = mrqq->pop();

    // Power stats
    // if(req->data->get_type() != READ_REPLY && req->data->get_type() !=
    // WRITE_ACK)
    m_stats->total_n_access++;

    if (req->data->get_type() == WRITE_REQUEST) {
      m_stats->total_n_writes++;
    } else if (req->data->get_type() == READ_REQUEST) {
      m_stats->total_n_reads++;
    }

    req->data->set_status(IN_PARTITION_MC_INPUT_QUEUE,
                          m_gpu->gpu_sim_cycle + m_gpu->gpu_tot_sim_cycle);
    sched->add_req(req);
  } // mrqq에 있는 모든 req를 schedular에 담는다.

  dram_req_t *req;
  unsigned i;
  for (i = 0; i < m_config->nbk; i++) {
    unsigned b = (i + prio) % m_config->nbk;
    if (!bk[b]->mrq) {
      req = sched->schedule(b, bk[b]->curr_row);
      //req 선택

      if (req) {
        req->data->set_status(IN_PARTITION_MC_BANK_ARB_QUEUE,
                              m_gpu->gpu_sim_cycle + m_gpu->gpu_tot_sim_cycle);
        prio = (prio + 1) % m_config->nbk;
        bk[b]->mrq = req;
        if (m_config->gpgpu_memlatency_stat) {
          mrq_latency = m_gpu->gpu_sim_cycle + m_gpu->gpu_tot_sim_cycle -
                        bk[b]->mrq->timestamp;
          m_stats->tot_mrq_latency += mrq_latency;
          m_stats->tot_mrq_num++;
          bk[b]->mrq->timestamp =
              m_gpu->gpu_tot_sim_cycle + m_gpu->gpu_sim_cycle;
          m_stats->mrq_lat_table[LOGB2(mrq_latency)]++;
          if (mrq_latency > m_stats->max_mrq_latency) {
            m_stats->max_mrq_latency = mrq_latency;
          }
        }

        break;
      }
    }
  }
}
