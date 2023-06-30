#include "atpg.h"

void ATPG::test_tdfatpg(){
    srand(0);
    clock_t timer;
    float total_time = 0.0;
    timer = clock();
    SCOAP_ctrlability();
    SCOAP_obsrability();
    int total_detect_num = 0;
    int current_detected = 0;
    int drop = 0;
    bool detect_any = false;
    printf("-----------------This is tdfatpg mode-----------------\n");
    /*-----------------ATPG(DTC)-----------------*/
    tdf_podem();
    // unordered_set<string> s;
    // for(int i=0;i<vectors.size();i++){
    //   if(!(s.insert(vectors[i]).second)) {
    //     fprintf(stderr,"redundant pattern exist!\n");
    //   }
    // }

    if (get_compression())
    {
      /*-----------------STC-----------------*/
      STC();
      /*-----------------STC-----------------*/

      /*-----------------SIM-----------------*/
      total_detect_num = 0;
      current_detected = 0;
      drop = 0;
      detect_any = false;
      
      flist_undetect.clear();
      for (auto it = flist.begin(); it != flist.end(); ++it)
      {
        fptr f = (*it).get();
        f->detected_time = 0;
        f->detect = FALSE;
        flist_undetect.push_front(f);
      }
      for (const string &vec : vectors)
      {
        current_detected = 0;
        drop = 0;
        tdfault_sim_a_vector(vec, current_detected, drop, detect_any);
        total_detect_num += drop;
      }
      display_test_patterns();

      fprintf(stderr,"compression: %d      N: %d \n", (int)get_compression(),detected_num);
      fprintf(stderr,"# number of test patterns after stc = %lu\n", vectors.size());
      fprintf(stderr, "# number of detected faults after stc = %d\n", total_detect_num);
      fprintf(stderr, "# number of total faults = %d\n", num_of_tdf_fault);
      fprintf(stderr, "# FC after stc = %f %\n", double(total_detect_num) * 100 / double(num_of_tdf_fault));

      fprintf(stderr, "-----------------End of tdf ATPG-----------------\n");
      /*-----------------SIM-----------------*/
    }
    else
    {
      /*-----------------SIM-----------------*/
      total_detect_num = 0;
      current_detected = 0;
      drop = 0;
      detect_any = false;
      // display_test_patterns();
      flist_undetect.clear();
      for (auto it = flist.begin(); it != flist.end(); ++it)
      {
        fptr f = (*it).get();
        f->detected_time = 0;
        f->detect = FALSE;
        flist_undetect.push_front(f);
      }
      for (const string &vec : vectors)
      {
        current_detected = 0;
        drop = 0;
        tdfault_sim_a_vector(vec, current_detected, drop, detect_any);
        total_detect_num += drop;
      }
      display_test_patterns();
      fprintf(stderr,"compression: %d      N: %d \n", (int)get_compression(),detected_num);
      fprintf(stderr,"# number of test patterns = %lu\n", vectors.size());
      fprintf(stderr, "# number of detected faults = %d\n", total_detect_num);
      fprintf(stderr, "# number of total faults = %d\n", num_of_tdf_fault);
      fprintf(stderr, "# FC = %f %\n", double(total_detect_num) * 100 / double(num_of_tdf_fault));

      fprintf(stderr, "-----------------End of tdf ATPG-----------------\n");
      /*-----------------SIM-----------------*/
    }
    timer = clock()-timer;
    total_time += (float)timer/CLOCKS_PER_SEC;
    fprintf(stderr, "Time: %f sec(s)\n", (float)timer/CLOCKS_PER_SEC);
}



void ATPG::display_test_patterns() const {
  int i, j;

  for (i = 0; i < vectors.size(); i++) {
    const string& vec = vectors[i];
    fprintf(stdout, "T\'");
    for (j = 0; j < cktin.size(); ++j) {
      fprintf(stdout, "%c", vec[j]);
    }
    fprintf(stdout, " %c\'\n", vec[cktin.size()]);
  }
}


