/**********************************************************************/
/*           Transition Delay Fault ATPG:                             */
/*           podem-x implementation                                   */
/*                                                                    */
/*           Author: Hsiang-Ting Wen, Fu-Yu Chuang, and Chen-Hao Hsu  */
/*           Date : 12/27/2018 created                                */
/**********************************************************************/

#include "atpg.h"
#define CONFLICT 2
int ATPG::tdf_podem()
{
    int current_detect_num = 0;
    int current_drop_num = 0;
    int total_backtracks = 0;
    int current_backtracks = 0;
    int aborted_fnum = 0;
    int redundant_fnum = 0;
    int call_num = 0;
    int drop = 0;
    bool detec_fsim;

    int v2_loop_limit = 20;
    int secondary_limit = 20;
    
    /* declare parameters */
    forward_list<wptr>   decision_tree;           
    string decision_tree_flag_prim;     
    string tdf_vec, tmp_vec;          
    fptr   primary_fault, secondary_fault;   
    unordered_set<string> set; 

    bool   test_found;       
    bool   need_v2;       
    bool   v1_not_found;    
    bool   primary;       
    
    int    v2_loop_counter;
    int    secondary_counter;

    // initialize_fault_primary_record();
    tdf_vec.resize(cktin.size()+1, '2');
    decision_tree_flag_prim.resize(cktin.size(), '0');
    for (fptr fptr_ele: flist_undetect) {
        fptr_ele->decision_tree_prim = decision_tree;
        fptr_ele->allAssigned_prim = decision_tree_flag_prim;
        fptr_ele->primary_vector = tdf_vec;
    }
    

    primary_fault = choose_primary_fault(flist_undetect);
    
    while (primary_fault != nullptr ) {  

        test_found = false;
        need_v2 = true;
        v1_not_found = false;
        primary = true;
        v2_loop_counter = 0;
        tdf_vec.resize(cktin.size()+1, '2');
        decision_tree_flag_prim.resize(cktin.size(), '0');
        
       
        
        while ((!test_found) && (v2_loop_counter < v2_loop_limit) ) {

            if (need_v2) {
                int find_v2 = tdf_find_v2(primary_fault, current_backtracks, tdf_vec, true,decision_tree_flag_prim,v1_not_found,decision_tree);
                if (find_v2 == TRUE) {
                    need_v2 = false;
                    v2_loop_counter++;
                } 
                else  {
                    if(find_v2 == FALSE) redundant_fnum+=1;
                    break; 
                }
            }

            else {
                forward_list<wptr> tmp_decision_tree;         
                tmp_vec = tdf_vec;
                int find_v1 = tdf_find_v1(primary_fault, current_backtracks, tmp_vec, tmp_decision_tree);
                if (find_v1 == TRUE) {
                    tdf_vec = tmp_vec;
                    test_found = true;
                } else {
                    if(find_v1 == FALSE) redundant_fnum+=1;
                    need_v2 = true;
                    v1_not_found = true;
                }
            }
        }

        bool test_found_primary = test_found;
        
        if (get_compression() && test_found && enough_x(tdf_vec)) { 
            string tdf_vector_last;
            secondary_counter = 0;
            do {
                tdf_vector_last = tdf_vec;

                secondary_fault = choose_secondary_fault(flist_undetect,primary_fault);
                secondary_counter++;
                

                test_found = false;
                need_v2 = true;
                v1_not_found = false;
                primary = false;
                v2_loop_counter = 0;
                

                decision_tree.clear();
                decision_tree_flag_prim.resize(cktin.size(), '0');
                

                if (secondary_fault != nullptr) {
                    while (!test_found  && (v2_loop_counter < v2_loop_limit)) {

                        if (need_v2) {
                            int find_v2 = tdf_find_v2(secondary_fault, current_backtracks, tdf_vec, false,decision_tree_flag_prim,v1_not_found,decision_tree);
                            if (find_v2 == TRUE) {
                                need_v2 = false;
                                v2_loop_counter++;
                            } 
                            else break;
                        }

                        else {
                            forward_list<wptr> tmp_decision_tree;
                            tmp_vec = tdf_vec;
                            int find_v1 = tdf_find_v1(secondary_fault, current_backtracks, tmp_vec, tmp_decision_tree);
                            if (find_v1 == TRUE) {
                                tdf_vec = tmp_vec;
                                test_found = true;
                            } else {
                                need_v2 = true;
                                v1_not_found = true;
                            }
                        }
                    }
                }
            } while ((secondary_fault != nullptr) && (secondary_counter < secondary_limit) && enough_x(tdf_vec));
            if(!enough_x(tdf_vec)) tdf_vec = tdf_vector_last;
        }

        if (test_found_primary) {
            vector<string> expanded_patterns;
            expand_pattern(expanded_patterns, tdf_vec);
            int detect_time = primary_fault->detected_time;
            if(detect_time < detected_num) {

                for (string pattern : expanded_patterns) {
                    if(!(set.insert(pattern).second)) continue;
                    vectors.emplace_back(pattern);
                    tdfault_sim_a_vector(vectors.back(), current_detect_num,drop,detec_fsim);
                    detect_time++;
                    if (detect_time >= detected_num) break;
                    
                }
            }
        }      
        

        primary_fault->test_tried = true;
        primary_fault = choose_primary_fault(flist_undetect);
        total_backtracks += current_backtracks;
        call_num++;
    } 

    fprintf(stdout,"\n");
    fprintf(stdout,"#number of aborted faults = %d\n",aborted_fnum);
    fprintf(stdout,"\n");
    fprintf(stdout,"#number of redundant faults = %d\n",redundant_fnum);
    fprintf(stdout,"\n");
    fprintf(stdout,"#number of calling podem1 = %d\n",call_num);
    fprintf(stdout,"\n");
    fprintf(stdout,"#total number of backtracks = %d\n",total_backtracks);
    return 0;
}

// int ATPG::count_unknown_input(string tdf_vec){
//     int counter = 0;
//     for(int i = 0; i< tdf_vec.size(); i++){
//         if(tdf_vec[i] = '2') counter++;
//     }
//     return counter;
// }
int ATPG::tdf_find_v2(fptr target_fault,int &current_backtracks,string& tdf_vec, bool primary, string& decision_tree_flag, bool not_first_time, forward_list<wptr>& decision_tree){
    int i, ncktwire, ncktin;
    wptr wpi;                         // points to the PI currently being assigned
    //forward_list<wptr> decision_tree; // design_tree (a LIFO stack)
    wptr wfault;
    string vec_v2;
    int attempt_num = 0;
    int obj;
    ncktwire = sort_wlist.size();
    ncktin = cktin.size();
    vec_v2.resize(ncktin+1,'2');
    
    
    if (primary && (!not_first_time)) {
        decision_tree = target_fault->decision_tree_prim;
        tdf_vec = target_fault->primary_vector;
        decision_tree_flag = target_fault->allAssigned_prim;
    }
    vec_v2 = tdf_vec;
    // cout << "tdf"<<tdf_vec << "\n";
    // cout << "vec"<<vec_v2 << "\n";
    for (i = 0; i < ncktwire; i++)
    {
        sort_wlist[i]->value = U;
    }
    //restore vector
    for (int i = 0; i < ncktin; i++) {
        // int vec_index = (i == 0) ? ncktin : (i-1); //different
        int vec_index;
        if(i == 0) vec_index = ncktin;
        else vec_index = (i-1);
        switch(vec_v2[vec_index]) {
            case '0':
                cktin[i]->value = 0;
                cktin[i]->set_changed();
                break;
            case '1':
                cktin[i]->value = 1;
                cktin[i]->set_changed();
                break;
            case '2':
                cktin[i]->value = U;
                cktin[i]->set_changed();
                break;
        }
    }
    sim();

    //restore all assigned
    for (int i = 0; i < ncktin; ++i) {
        if (decision_tree_flag[i] == '1') {
            cktin[i]->set_all_assigned();
        } else {
            cktin[i]->remove_all_assigned();
        }
    }
    
    mark_propagate_tree(target_fault->node);
    no_of_backtracks = 0;
    find_test = false;
    no_test = false;
    if(!not_first_time){
        switch (set_uniquely_implied_value(target_fault))
        {
        case TRUE:     // if a  PI is assigned
                sim(); // Fig 7.3
                wfault = fault_evaluate(target_fault);
                if (wfault != nullptr)
                    forward_imply(wfault); // propagate fault effect
                if (check_test())
                    find_test = true; // if fault effect reaches PO, done. Fig 7.10
                break;
        case CONFLICT:
                no_test = true; // cannot achieve initial objective, no test
                break;
        case FALSE:
                break; // if no PI is reached, keep on backtracing. Fig 7.A
        }
    }
    

    while ((no_of_backtracks <= backtrack_limit) && (!no_test) &&
           (!find_test))
    {
            
            /* check if test possible.   Fig. 7.1 */
            if (wpi = test_possible_2(target_fault,obj))
            {
                wpi->value = obj;
                wpi->set_changed();  //need to be viewed
                /* insert a new PI into decision_tree */
                decision_tree.push_front(wpi);
            }
            else
            { // no test possible using this assignment, backtrack.

                while (!decision_tree.empty() && (wpi == nullptr))
                {
                    /* if both 01 already tried, backtrack. Fig.7.7 */
                    if (decision_tree.front()->is_all_assigned())
                    {
                        decision_tree.front()->remove_all_assigned(); // clear the ALL_ASSIGNED flag
                        decision_tree.front()->value = U;             // do not assign 0 or 1
                        decision_tree.front()->set_changed();         // this PI has been changed
                        /* remove this PI in decision tree.  see dashed nodes in Fig 6 */
                        decision_tree.pop_front();
                    }
                    /* else, flip last decision, flag ALL_ASSIGNED. Fig. 7.8 */
                    else
                    {
                        decision_tree.front()->value = decision_tree.front()->value ^ 1; // flip last decision
                        decision_tree.front()->set_changed();                            // this PI has been changed
                        decision_tree.front()->set_all_assigned();
                        no_of_backtracks++;
                        wpi = decision_tree.front();
                    }
                } // while decision tree && ! wpi
                if (wpi == nullptr)
                    no_test = true; // decision tree empty,  Fig 7.9
            }                       // no test possible

            if (wpi)
            {
                sim();
                if (wfault = fault_evaluate(target_fault))
                    forward_imply(wfault);
                if (check_test())
                {
                    find_test = true;
                }
            }
    // again:
    //         if (wpi)
    //         {
    //             sim();
    //             if (wfault = fault_evaluate(target_fault))
    //                 forward_imply(wfault);
    //             if (check_test())
    //             {
    //                 find_test = true;
    //                 /* if multiple patterns per fault, print out every test cube */
    //                 if (total_attempt_num > 1)
    //                 {
    //                     if (attempt_num == 0)
    //                     {
    //                         display_fault(target_fault);
    //                     }
    //                 }
    //                 attempt_num++; // increase pattern count for this fault

    //                 /* keep trying more PI assignments if we want multiple patterns per fault
    //                  * this is not in the original PODEM paper*/
    //                 if (total_attempt_num > attempt_num)
    //                 {
    //                     wpi = nullptr;
    //                     while (!decision_tree.empty() && (wpi == nullptr))
    //                     {
    //                         /* backtrack */
    //                         if (decision_tree.front()->is_all_assigned())
    //                         {
    //                             decision_tree.front()->remove_all_assigned();
    //                             decision_tree.front()->value = U;
    //                             decision_tree.front()->set_changed();
    //                             decision_tree.pop_front();
    //                         }
    //                         /* flip last decision */
    //                         else
    //                         {
    //                             decision_tree.front()->value = decision_tree.front()->value ^ 1;
    //                             decision_tree.front()->set_changed();
    //                             decision_tree.front()->set_all_assigned();
    //                             no_of_backtracks++;
    //                             wpi = decision_tree.front();
    //                         }
    //                     }
    //                     if (!wpi)
    //                         no_test = true;
    //                     goto again; // if we want multiple patterns per fault
    //                 }               // if total_attempt_num > attempt_num
    //             }                   // if check_test()
    //         }                       // again
    }                               // while (three conditions)

   
    if (find_test)
    {
        if (primary)
        {
                target_fault->decision_tree_prim = decision_tree;

                for (int i = 0; i < ncktin; i++)
                {
                    target_fault->allAssigned_prim[i] = (cktin[i]->is_all_assigned()) ? '1' : '0';
                }

                for (i = 0; i < ncktin; i++)
                {
                    // int vec_index = (i == 0) ? ncktin : (i - 1);
                    int vec_index;
                    if(i == 0) vec_index = ncktin;
                    else vec_index = (i - 1);
                    switch (cktin[i]->value)
                    {
                    case 1:
                    case D:
                        target_fault->primary_vector[vec_index] = '1';
                        break;
                    case 0:
                    case D_bar:
                        target_fault->primary_vector[vec_index] = '0';
                        break;
                    case U:
                        target_fault->primary_vector[vec_index] = '2';
                        break;
                    }
                }
        }
        //extract all assigned
        for (int i = 0; i < ncktin; i++)
        {
            decision_tree_flag[i] = (cktin[i]->is_all_assigned()) ? '1' : '0';
        }
        
    }
    for (wptr wptr_ele : decision_tree)
    {
        wptr_ele->remove_all_assigned();
    }
    //decision_tree.clear();

    current_backtracks = no_of_backtracks;
    unmark_propagate_tree(target_fault->node);
    if(find_test){
        for (i = 0; i < ncktin; i++)
        {
            int vec_index = (i == 0) ? ncktin : (i-1);
                switch (cktin[i]->value)
                {
                case 1:
                case D:
                    vec_v2[vec_index] = '1';
                    break;
                case 0:
                case D_bar:
                    vec_v2[vec_index] = '0';
                    break;
                case U:
                    vec_v2[vec_index] = '2';
                    break; // random fill U
                }
        }
      tdf_vec = vec_v2;
    //   cout << vec_v2 << "\n";
    //   cout << tdf_vec << "\n";
    //   cout << "v2\n";
    //   display_io();
      //initial
      for (int i = 0; i < ncktin; ++i)
      {
            cktin[i]->value = U;
            cktin[i]->set_changed();
      }
      sim();
      return (TRUE);
    }
    else if (no_test)
    {
      /*fprintf(stdout,"redundant fault...\n");*/
      return (FALSE);
    }
    else
    {
      /*fprintf(stdout,"test aborted due to backtrack limit...\n");*/
      return (MAYBE);
    }
}
ATPG::wptr ATPG::test_possible_2(const fptr fault,int& object_level) {
  nptr n;
  wptr object_wire;
//   int object_level;

  /* if the fault is not on primary output */
  if (fault->node->type != OUTPUT) {

    /* if the faulty gate (aka. gate under test, G.U.T.) output is not U,  Fig. 8.1 */
    if (fault->node->owire.front()->value ^ U) {

      /* if GUT output is not D or D_bar, no test possible */
      if (!((fault->node->owire.front()->value == D_bar) ||
            (fault->node->owire.front()->value == D)))
        return (nullptr);

      /* find the next gate n to propagate, Fig 8.5*/
      if (!(n = find_propagate_gate(fault->node->owire.front()->level)))
        return (nullptr);

      /*determine objective level according to the type of n.   Fig 8.8*/
      switch (n->type) {
        case AND:
        case NOR:
          object_level = 1;
          break;
        case NAND:
        case OR:
          object_level = 0;
          break;
        default:
          // to be improved
          /*---- comment out because C2670 has XOR ---------
          fprintf(stderr,
              "Internal Error(1st bp. in test_possible)!\n");
          exit(-1);
          -------------------------------------------------------*/
          return (nullptr);
      }
      /* object_wire is the gate n output. */
      object_wire = n->owire.front();
    }  // if faulty gate output is not U.   (fault->node->owire.front()->value ^ U)

    else { // if faulty gate output is U

      /* if X path disappear, no test possible  */
      if (!(trace_unknown_path(fault->node->owire.front())))
        return (nullptr);

      /* if fault is on GUT otuput,  Fig 8.2*/
      if (fault->io) {
        /* objective_level is opposite to stuck fault  Fig 8.3 */
        if (fault->fault_type) object_level = 0;
        else object_level = 1;
        /* objective_wire is on faulty gate output */
        object_wire = fault->node->owire.front();
      }

        /* if fault is on GUT input, Fig 8.2*/
      else {
        /* if faulted input is not U  Fig 8.4 */
        if (fault->node->iwire[fault->index]->value ^ U) {
          /* determine objective value according to GUT type. Fig 8.9*/
          switch (fault->node->type) {
            case AND:
            case NOR:
              object_level = 1;
              break;
            case NAND:
            case OR:
              object_level = 0;
              break;
            default:
              // to be improved
              /*---- comment out because C2670.sim has XOR ---------
                 fprintf(stderr,
                    "Internal Error(2nd bp. in test_possible)!\n");
                 exit(-1);
              -------------------------------------------------------*/
              return (nullptr);
          }
          /*objective wire is GUT output. */
          object_wire = fault->node->owire.front();
        }  // if faulted input is not U

        else { // if faulted input is U
          /*objective level is opposite to stuck fault.  Fig 8.10*/
          if (fault->fault_type) object_level = 0;
          else object_level = 1;
          /* objective wire is faulty wire itself */
          object_wire = fault->node->iwire[fault->index];
        }
      }
    }
  } // if fault not on PO

  else { // else if fault on PO
    /* if faulty PO is still unknown */
    if (fault->node->iwire.front()->value == U) {
      /*objective level is opposite to the stuck fault */
      if (fault->fault_type) object_level = 0;
      else object_level = 1;
      /* objective wire is the faulty wire itself */
      object_wire = fault->node->iwire.front();
    } else {
      /*--- comment out due to error for C2670.sim ---
      fprintf(stderr,"Internal Error(1st bp. in test_possible)!\n");
      exit(-1);
        */
      return (nullptr);
    }
  }// else if fault on PO

  /* find a pi to achieve the objective_level on objective_wire.
   * returns nullptr if no PI is found.  */
  return (find_pi_assignment_2(object_wire, object_level));
}/* end of test_possible */


int ATPG::tdf_find_v1(fptr target_fault, int &current_backtracks, string& tdf_vec,forward_list<wptr>& decision_tree)
{

    int i, ncktwire, ncktin;
    wptr wpi;
    wptr wfault = sort_wlist[target_fault->to_swlist];
    vector<wptr> fanin_list;
    ncktwire = sort_wlist.size();
    ncktin = cktin.size();

    //restore v1
    for (int i = 0; i < ncktwire; i++) {
        sort_wlist[i]->value = U;
    }
    for (int i = 0; i < ncktin; i++) {
        switch(tdf_vec[i]) {
            case '0':
                cktin[i]->value = 0;
                cktin[i]->set_changed();
                break;
            case '1':
                cktin[i]->value = 1;
                cktin[i]->set_changed();
                break;
            case '2':
                cktin[i]->value = U;
                cktin[i]->set_changed();
                break;
        }
    }

    sim();

    find_test = false;
    no_test = false;
    current_backtracks = 0;

    mark_fanin(wfault, fanin_list);
    for (wptr w : fanin_list) {
        w->remove_marked();
    }
    
    auto partial_sim = [&] (vector<wptr> &wlist) -> void {
        for (int i = 0, nwire = wlist.size(); i < nwire; ++i) {
            if (wlist[i]->is_input()) continue;
                evaluate(wlist[i]->inode.front());
            }
        return;
    };

    partial_sim(fanin_list);
    
   
    if (wfault->value != U) {
        if(wfault->value == target_fault->fault_type) return TRUE;
        else return FALSE;
    }


        while ((current_backtracks <= 100) && (!no_test) && (!find_test)) {
        int obj_value = target_fault->fault_type;
        wpi = find_pi_assignment_2(wfault, obj_value);
        
        if (wpi != nullptr) {
            wpi->value = obj_value;
            wpi->set_changed();
            decision_tree.push_front(wpi);
        }
        
        else {
            while (!decision_tree.empty() && (wpi == nullptr)) {
                if (decision_tree.front()->is_all_assigned()) {
                    decision_tree.front()->remove_all_assigned();
                    decision_tree.front()->value = U;
                    decision_tree.front()->set_changed();
                    decision_tree.pop_front();
                }
        
                else {
                    decision_tree.front()->value = decision_tree.front()->value ^ 1;
                    decision_tree.front()->set_changed();
                    decision_tree.front()->set_all_assigned();
                    current_backtracks++;
                    wpi = decision_tree.front();
                }
            }
            if (wpi == nullptr) {
                no_test = true;
            }
        }
       
        if (wpi != nullptr) {
            partial_sim(fanin_list);
            if (wfault->value == target_fault->fault_type) {
                find_test = true;
            }
        }
    } // while (three conditions)

    for (int i = 0; i < ncktin; ++i) {
        cktin[i]->remove_marked();
        cktin[i]->remove_changed();
        cktin[i]->remove_all_assigned();
    }

    if (find_test) {
        for (i = 0; i < ncktin; i++)
        {
                switch (cktin[i]->value)
                {
                case 1:
                case D:
                    tdf_vec[i] = '1';
                    break;
                case 0:
                case D_bar:
                    tdf_vec[i] = '0';
                    break;
                case U:
                    tdf_vec[i] = '2';
                    break; 
                }
        }
        return (TRUE);
    } else if (no_test) {
        return (FALSE);
    } else {
        return (MAYBE);
    }
}

void ATPG::mark_fanin(const wptr w, vector<wptr> &fanin_cone_wlist) {
    if (w->is_marked()) {
        return;
    }
    int i, j, ninode, niwire;
    w->set_marked();
    for (i = 0, ninode = w->inode.size(); i < ninode; ++i) {
        for (j = 0, niwire = w->inode[i]->iwire.size(); j < niwire; ++j) {
            mark_fanin(w->inode[i]->iwire[j], fanin_cone_wlist);
        }
    }
    fanin_cone_wlist.emplace_back(w);
}

// bool ATPG::find_objective(const fptr fault, wptr& obj_wire, int& obj_value)
// {
//     nptr n;
//     /* if the fault is not on PO */
//     if (fault->node->type != OUTPUT) {

//         /* if the faulty gate output is not U, it should be D or D' */
//         if (fault->node->owire.front()->value ^ U) {
//             if (!((fault->node->owire.front()->value == D_bar) || (fault->node->owire.front()->value == D))) {
//                 return false;
//             }
//             /* find the next gate n to propagate */
//             if (!(n = find_propagate_gate(fault->node->owire.front()->level))) {
//                 return false;
//             }
//             /* determine objective level according to the type of n. */
//             switch(n->type) {
//                 case  AND:
//                 case  NOR:
//                     obj_value = 1;
//                     break;
//                 case NAND:
//                 case   OR:
//                     obj_value = 0;
//                     break;
//                 default:
//                     return false;
//             }
//             obj_wire = n->owire.front();
//         } else { // if faulty gate output is U

//             /* if X path disappear, return false  */
//             if (!(trace_unknown_path(fault->node->owire.front()))) {
//                 return false;
//             }

//             /* if fault is a GO fault */
//             if (fault->io) {
//                 obj_value = (fault->fault_type) ? 0 : 1;
//                 obj_wire = fault->node->owire.front();
//             }
//             /* if fault is a GI fault */
//             else {
//                 /* if faulted input is not U */
//                 if (fault->node->iwire[fault->index]->value ^ U) {
//                     /* determine objective value according to GUT type. Fig 8.9*/
//                     switch (fault->node->type) {
//                         case  AND:
//                         case  NOR:
//                             obj_value = 1;
//                             break;
//                         case NAND:
//                         case   OR:
//                             obj_value = 0;
//                             break;
//                         default:
//                             return false;
//                     }
//                     obj_wire = fault->node->owire.front();
//                 } else { // if faulted input is U
//                     obj_value = (fault->fault_type) ? 0 : 1;
//                     obj_wire = fault->node->iwire[fault->index];
//                 }
//             }
//         }
//     }
//     /* if the fault is on PO */
//     else {
//         /* if faulty PO is still unknown */
//         if (fault->node->iwire.front()->value == U) {
//             /*objective level is opposite to the stuck fault */
//             obj_value = (fault->fault_type) ? 0 : 1;
//             obj_wire = fault->node->iwire.front();
//         } else {
//             return false;
//         }
//     }
//     return true;
// }

ATPG::wptr ATPG::find_pi_assignment_2(const wptr obj_wire, int& obj_value)
{
    wptr new_obj_wire;

    /* if PI, assign the same value as objective */
    if (obj_wire->is_input()) {
        return obj_wire;
    }

    /* if not PI, backtrace to PI */
    else {
        switch(obj_wire->inode.front()->type) {
            case   OR:
            case NAND:
                // if (obj_value) new_obj_wire = find_easiest_control(obj_wire->inode.front());  // decision gate
                // else new_obj_wire = find_hardest_control(obj_wire->inode.front()); // imply gate
                if (obj_value) new_obj_wire = find_easiest_control_scoap(obj_wire->inode.front(),obj_value);  // decision gate
                else new_obj_wire = find_hardest_control_scoap(obj_wire->inode.front(),obj_value); // imply gate
               
                break;
            case  NOR:
            case  AND:
                // if (obj_value) new_obj_wire = find_hardest_control(obj_wire->inode.front());  // decision gate
                // else new_obj_wire = find_easiest_control(obj_wire->inode.front()); // imply gate
                if (obj_value) new_obj_wire = find_hardest_control_scoap(obj_wire->inode.front(),obj_value);  // decision gate
                else new_obj_wire = find_easiest_control_scoap(obj_wire->inode.front(),obj_value); // imply gate

                break;
            case  NOT:
            case  BUF:
                new_obj_wire = obj_wire->inode.front()->iwire.front();
                break;
            default:
                fprintf(stderr,"Something wrong in find_cool_pi(const wptr obj_wire, int& obj_value)\n");
                new_obj_wire = nullptr;
                break;
        }

        switch (obj_wire->inode.front()->type) {
            case  NOT:
            case  NOR:
            case NAND:
                obj_value = obj_value ^ 1;
                break;
            default:
                break;
        }
        if (new_obj_wire) {
            return find_pi_assignment_2(new_obj_wire, obj_value);
        }
        else {
            return nullptr;
        }
    }
}



bool ATPG::enough_x(const string& pattern) const
{
    int required_x_bit = (int)ceil(log2((double)detected_num));
    int x_bit_count = x_count(pattern);
    return (x_bit_count > required_x_bit);
}

void ATPG::expand_pattern(vector<string>& expanded_patterns, const string& pattern)
{
    int x_bit_count = x_count(pattern);

    size_t pos = pattern.find_first_of('2');
    if (x_bit_count <= 3 && pos != string::npos) {
        expand_pattern_rec(expanded_patterns, pattern, '0', pos);
        expand_pattern_rec(expanded_patterns, pattern, '1', pos);
    } else if (x_bit_count > 3 && pos != string::npos) {
        expand_pattern_rec_limited(expanded_patterns, pattern, '0', pos, 0);
        expand_pattern_rec_limited(expanded_patterns, pattern, '1', pos, 0);
    } else {
        expanded_patterns.push_back(pattern);
    }
}

void ATPG::expand_pattern_rec(vector<string>& patterns, string pattern, char bit, size_t pos)
{
    pattern[pos] = bit;
    pos = pattern.find_first_of('2', pos+1);
    if (pos != string::npos) {
        expand_pattern_rec(patterns, pattern, '0', pos);
        expand_pattern_rec(patterns, pattern, '1', pos);
    } else {
        patterns.push_back(pattern);
    }
}

void ATPG::expand_pattern_rec_limited(vector<string>& patterns, string pattern, char bit, size_t pos, size_t depth)
{
    ++depth;
    pattern[pos] = bit;
    pos = pattern.find_first_of('2', pos+1);
    if (pos != string::npos && depth <= 2) { //changed
        expand_pattern_rec_limited(patterns, pattern, '0', pos, depth);
        expand_pattern_rec_limited(patterns, pattern, '1', pos, depth);
    } 
    // else {
    //     patterns.push_back(pattern);
    // }
    else {
        pos = pattern.find_first_of('2',pos);
        while(pos != string::npos){
            pattern[pos] =itoc(rand() % 2);
            pos = pattern.find_first_of('2',pos+1);
        }
        patterns.push_back(pattern);
    }
}

int ATPG::x_count(const string& pattern) const
{
    int ret = 0;
    for (size_t i = 0; i < pattern.size(); ++i) {
        if (pattern[i] == '2') {
            ++ret;
        }
    }
    return ret;
}


ATPG::fptr ATPG::choose_primary_fault(forward_list<fptr> flist_sorted)
{
  fptr fault_return;
  for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
    fptr f;
    f = *pos;
    if (!f->test_tried) {
      fault_return = f;
      return fault_return;
    }
  }
  return nullptr;

//   vector<fptr> prim_flist;
//   for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
//     fptr f = *pos;
//     prim_flist.push_back(f);
//   }
//   if(prim_flist.empty()) return nullptr;
//   random_shuffle(prim_flist.begin(),prim_flist.end());

//   return prim_flist.front();
}

ATPG::fptr ATPG::choose_secondary_fault(forward_list<fptr> flist_sorted, fptr primary_fault)
{
//   fptr fault_return;
//   for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
//     fptr f;
//     f = *pos;
//     if (!f->test_tried && f!=primary_fault) {
//       fault_return = f;
//       return fault_return;
//     }
//   }
//   return nullptr;
  vector<fptr> second_flist;
  for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
    fptr f = *pos;
    second_flist.push_back(f);
  }
  if(second_flist.empty()) return nullptr;
  random_shuffle(second_flist.begin(),second_flist.end());

  return second_flist.front();
}


// **************************************************************************
// Function   [ Atpg::SCOAP_ctrlability ]
// Usage      [ determine SCOAP controllability for combinational ckt：CC0 CC1]
// Date       [ Kai-Wei Lin 2023/05/17 v_1 ]
// **************************************************************************
void ATPG::SCOAP_ctrlability()
{
  int wire_num, node_type, node_iwire_num;
  int cc0, cc1;
  nptr node;

  wire_num = sort_wlist.size();

  for (int i = 0; i < wire_num; ++i)
  {
    if (sort_wlist[i]->is_input())
    {
      sort_wlist[i]->cc0 = 1;
      sort_wlist[i]->cc1 = 1;
    }
    node = sort_wlist[i]->inode.front();
    node_type = node->type;
    node_iwire_num = node->iwire.size();

    switch (node_type)
    {
    case AND:
      cc0 = node->iwire.front()->cc0;
      cc1 = 1;
      for (int j = 0; j < node_iwire_num; ++j)
      {
        if (node->iwire[j]->cc0 < cc0)
          cc0 = node->iwire[j]->cc0;
        cc1 += node->iwire[j]->cc1;
      }
      sort_wlist[i]->cc0 = cc0 + 1;
      sort_wlist[i]->cc1 = cc1;
      break;
    case OR:
      cc0 = 1;
      cc1 = node->iwire.front()->cc1;
      for (int j = 0; j < node_iwire_num; ++j)
      {
        if (node->iwire[j]->cc1 < cc1)
          cc1 = node->iwire[j]->cc1;
        cc0 += node->iwire[j]->cc0;
      }
      sort_wlist[i]->cc0 = cc0;
      sort_wlist[i]->cc1 = cc1 + 1;
      break;
    case XOR:
    case NOT:
      sort_wlist[i]->cc0 = node->iwire.front()->cc1 + 1;
      sort_wlist[i]->cc1 = node->iwire.front()->cc0 + 1;
      break;
    case BUF:
      sort_wlist[i]->cc0 = node->iwire.front()->cc0 + 1;
      sort_wlist[i]->cc1 = node->iwire.front()->cc1 + 1;
      break;
    case NAND:
      cc0 = node->iwire.front()->cc0;
      cc1 = 1;
      for (int j = 0; j < node_iwire_num; ++j)
      {
        if (node->iwire[j]->cc0 < cc0)
          cc0 = node->iwire[j]->cc0;
        cc1 += node->iwire[j]->cc1;
      }
      sort_wlist[i]->cc0 = cc1;
      sort_wlist[i]->cc1 = cc0 + 1;
      break;
    case NOR:
      cc0 = 1;
      cc1 = node->iwire.front()->cc1;
      for (int j = 0; j < node_iwire_num; ++j)
      {
        if (node->iwire[j]->cc1 < cc1)
          cc1 = node->iwire[j]->cc1;
        cc0 += node->iwire[j]->cc0;
      }
      sort_wlist[i]->cc0 = cc1 + 1;
      sort_wlist[i]->cc1 = cc0;
      break;
    }
    // fprintf(stderr, "# number of wire = %d\n", i);
    // fprintf(stderr, "# number of CC0 = %d\n", sort_wlist[i]->cc0);
    // fprintf(stderr, "# number of CC1 = %d\n", sort_wlist[i]->cc1);
  }
}
// **************************************************************************
// Function   [ Atpg::SCOAP_ctrlability ]
// Usage      [ determine SCOAP controllability for combinational ckt：CC0 CC1]
// Date       [ Kai-Wei Lin 2023/05/17 v_1 ]
// **************************************************************************
void ATPG::SCOAP_obsrability()
{
  int wire_num, node_type, wire_onode_num, node_iwire_num;
  int co, co_temp;
  nptr node;

  wire_num = sort_wlist.size();

  for (int i = wire_num - 1; i >= 0; --i)
  {
    int co_temp2 = 10000;
    if (sort_wlist[i]->is_output())
    {
      sort_wlist[i]->co = 0;
    }
    else
    {
      wire_onode_num = sort_wlist[i]->onode.size();
      for (int j = 0; j < wire_onode_num; ++j)
      {
        node = sort_wlist[i]->onode[j];
        node_type = node->type;
        co = node->owire.front()->co + 1;
        node_iwire_num = node->iwire.size();
        switch (node_type)
        {
        case AND:
          for (int k = 0; k < node_iwire_num; ++k)
          {
            if (node->iwire[k] != sort_wlist[i])
              co_temp = co + node->iwire[k]->cc1;
          }
        case OR:
          for (int k = 0; k < node_iwire_num; ++k)
          {
            if (node->iwire[k] != sort_wlist[i])
              co_temp = co + node->iwire[k]->cc0;
          }
        case XOR:
        case NOT:
        case NAND:
          for (int k = 0; k < node_iwire_num; ++k)
          {
            if (node->iwire[k] != sort_wlist[i])
              co_temp = co + node->iwire[k]->cc1;
          }
        case NOR:
          for (int k = 0; k < node_iwire_num; ++k)
          {
            if (node->iwire[k] != sort_wlist[i])
              co_temp = co + node->iwire[k]->cc0;
          }
        }
        if (co_temp < co_temp2)
          co_temp2 = co_temp;
      }
      sort_wlist[i]->co = co_temp2;
    }
    // fprintf(stderr, "# number of wire = %d\n", i);
    // fprintf(stderr, "# number of CO = %d\n", sort_wlist[i]->co);
  }
}

ATPG::wptr ATPG::find_hardest_control_scoap(const nptr n, const int value)
{
    int max = -1;
    wptr hardest_control = nullptr;
    int node_iwire_num;
    wptr w;
    node_iwire_num = n->iwire.size();
    for (int i = 0; i < node_iwire_num; ++i)
    {
        w = n->iwire[i];
        if (w->value == U)
        {
            if (value == 0)
            {
                hardest_control = (w->cc0 > max) ? w : hardest_control;
                max = (w->cc0 > max) ? w->cc0 : max;
            }
            else
            {
                hardest_control = (w->cc1 > max) ? w : hardest_control;
                max = (w->cc1 > max) ? w->cc1 : max;
            }
        }
    }

    return hardest_control;
}

ATPG::wptr ATPG::find_easiest_control_scoap(const nptr n, const int value)
{
    int min = 100000;
    wptr easiest_control = nullptr;
    int node_iwire_num;
    wptr w;
    node_iwire_num = n->iwire.size();
    for (int i = 0; i < node_iwire_num; ++i)
    {
        w = n->iwire[i];
        if (w->value == U)
        {
            if (value == 0)
            {
                easiest_control = (w->cc0 < min) ? w : easiest_control;
                min = (w->cc0 < min) ? w->cc0 : min;
            }
            else
            {
                easiest_control = (w->cc1 < min) ? w : easiest_control;
                min = (w->cc1 < min) ? w->cc1 : min;
            }
        }
    }

    return easiest_control;
}