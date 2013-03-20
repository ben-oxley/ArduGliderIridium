int get_Param_Key(char *buffer, int index)
{
  switch(droneType) {
#ifdef MAVLINK10
    case MAV_TYPE_FIXED_WING: { //MAV_FIXED_WING
#else // MAVLINK10
    case MAV_FIXED_WING: {  //MAV_FIXED_WING
#endif // MAVLINK10
      switch(index) {
        //SRV_RLL
        case 0: { strcpy_P(buffer, PSTR("RLL2SRV_P")); break; }
        case 1: { strcpy_P(buffer, PSTR("RLL2SRV_I")); break; }
        case 2: { strcpy_P(buffer, PSTR("RLL2SRV_D")); break; }
        case 3: { strcpy_P(buffer, PSTR("RLL2SRV_IMAX")); break; }

        //SRV_PIT
        case 4: { strcpy_P(buffer, PSTR("PTCH2SRV_P")); break; }
        case 5: { strcpy_P(buffer, PSTR("PTCH2SRV_I")); break; }
        case 6: { strcpy_P(buffer, PSTR("PTCH2SRV_D")); break; }
        case 7: { strcpy_P(buffer, PSTR("PTCH2SRV_IMAX")); break; }

        //SRV_YAW
        case 8: { strcpy_P(buffer, PSTR("YW2SRV_P")); break; }
        case 9: { strcpy_P(buffer, PSTR("YW2SRV_I")); break; }
        case 10: { strcpy_P(buffer, PSTR("YW2SRV_D")); break; }
        case 11: { strcpy_P(buffer, PSTR("YW2SRV_IMAX")); break; }
  
        //NAV_RLL
        case 12: { strcpy_P(buffer, PSTR("HDNG2RLL_P")); break; }
        case 13: { strcpy_P(buffer, PSTR("HDNG2RLL_I")); break; }
        case 14: { strcpy_P(buffer, PSTR("HDNG2RLL_D")); break; }
        case 15: { strcpy_P(buffer, PSTR("HDNG2RLL_IMAX")); break; }

        //NAV_PIT_Alt
        case 16: { strcpy_P(buffer, PSTR("ARSP2PTCH_P")); break; }
        case 17: { strcpy_P(buffer, PSTR("ARSP2PTCH_I")); break; }
        case 18: { strcpy_P(buffer, PSTR("ARSP2PTCH_D")); break; }
        case 19: { strcpy_P(buffer, PSTR("ARSP2PTCH_IMAX")); break; }

        //NAV_PIT_Alt
        case 20: { strcpy_P(buffer, PSTR("ALT2PTCH_P")); break; }
        case 21: { strcpy_P(buffer, PSTR("ALT2PTCH_I")); break; }
        case 22: { strcpy_P(buffer, PSTR("ALT2PTCH_D")); break; }
        case 23: { strcpy_P(buffer, PSTR("ALT2PTCH_IMAX")); break; }

        //Engery/Alt
        case 24: { strcpy_P(buffer, PSTR("ENRGY2THR_P")); break; }
        case 25: { strcpy_P(buffer, PSTR("ENRGY2THR_I")); break; }
        case 26: { strcpy_P(buffer, PSTR("ENRGY2THR_D")); break; }
        case 27: { strcpy_P(buffer, PSTR("ENRGY2THR_IMAX")); break; }

        //Other Mix
        case 28: { strcpy_P(buffer, PSTR("KFF_PTCH2THR")); break; }
        case 29: { strcpy_P(buffer, PSTR("KFF_PTCHCOMP")); break; }
        case 30: { strcpy_P(buffer, PSTR("KFF_RDDRMIX")); break; }
        case 31: { strcpy_P(buffer, PSTR("KFF_THR2PTCH")); break; }

        case 32: { strcpy_P(buffer, PSTR("XTRK_GAIN_SC")); break; }
        case 33: { strcpy_P(buffer, PSTR("XTRK_ANGLE_CD")); break; }
        case 34: { strcpy_P(buffer, PSTR("SONAR_ENABLE")); break; }
        case 35: { strcpy_P(buffer, PSTR("MAG_ENABLE")); break; }
        case 36: { strcpy_P(buffer, PSTR("ARSPD_ENABLE")); break; }
       
        default: { return -1; }
      } //end switch(index)
     } //end droneType == 1
     break;
    /*
#ifdef MAVLINK10
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_GENERIC: { //MAV_QUADROTOR
#else // MAVLINK10
    case MAV_QUADROTOR:
    case MAV_GENERIC: { // MAV_QUADROTOR
#endif // MAVLINK10

      switch (index) {
        case 0: { strcpy_P(buffer, PSTR("RATE_RLL_P")); break; }
        case 1: { strcpy_P(buffer, PSTR("RATE_RLL_I")); break; }
        case 2: { strcpy_P(buffer, PSTR("RATE_RLL_D")); break; }
        case 3: { strcpy_P(buffer, PSTR("RATE_RLL_IMAX")); break; }
        case 4: { strcpy_P(buffer, PSTR("RATE_PIT_P")); break; }
        case 5: { strcpy_P(buffer, PSTR("RATE_PIT_I")); break; }
        case 6: { strcpy_P(buffer, PSTR("RATE_PIT_D")); break; }
        case 7: { strcpy_P(buffer, PSTR("RATE_PIT_IMAX")); break; }
        case 8: { strcpy_P(buffer, PSTR("RATE_YAW_P")); break; }
        case 9: { strcpy_P(buffer, PSTR("RATE_YAW_I")); break; }
        case 10: { strcpy_P(buffer, PSTR("RATE_YAW_D")); break; }
        case 11: { strcpy_P(buffer, PSTR("RATE_YAW_IMAX")); break; }
        case 12: { strcpy_P(buffer, PSTR("STB_RLL_P")); break; }
        case 13: { strcpy_P(buffer, PSTR("STB_RLL_I")); break; }
        case 14: { strcpy_P(buffer, PSTR("STB_RLL_IMAX")); break; }
        case 15: { strcpy_P(buffer, PSTR("STB_PIT_P")); break; }
        case 16: { strcpy_P(buffer, PSTR("STB_PIT_I")); break; }
        case 17: { strcpy_P(buffer, PSTR("STB_PIT_IMAX")); break; }
        case 18: { strcpy_P(buffer, PSTR("STB_YAW_P")); break; }
        case 19: { strcpy_P(buffer, PSTR("STB_YAW_I")); break; }
        case 20: { strcpy_P(buffer, PSTR("STB_YAW_IMAX")); break; }
        case 21: { strcpy_P(buffer, PSTR("WP_SPEED_MAX")); break; }
        case 22: { strcpy_P(buffer, PSTR("NAV_LAT_P")); break; }
        case 23: { strcpy_P(buffer, PSTR("NAV_LAT_I")); break; }
        case 24: { strcpy_P(buffer, PSTR("NAV_LAT_D")); break; }
        case 25: { strcpy_P(buffer, PSTR("NAV_LAT_IMAX")); break; }
        case 26: { strcpy_P(buffer, PSTR("NAV_LON_P")); break; }
        case 27: { strcpy_P(buffer, PSTR("NAV_LON_I")); break; }
        case 28: { strcpy_P(buffer, PSTR("NAV_LON_D")); break; }
        case 29: { strcpy_P(buffer, PSTR("NAV_LON_IMAX")); break; }
        case 30: { strcpy_P(buffer, PSTR("THR_RATE_P")); break; }
        case 31: { strcpy_P(buffer, PSTR("THR_RATE_I")); break; }
        case 32: { strcpy_P(buffer, PSTR("THR_RATE_D")); break; }
        case 33: { strcpy_P(buffer, PSTR("THR_RATE_IMAX")); break; }
        case 34: { strcpy_P(buffer, PSTR("HLD_LAT_P")); break; }
        case 35: { strcpy_P(buffer, PSTR("HLD_LAT_I")); break; }
        case 36: { strcpy_P(buffer, PSTR("HLD_LAT_IMAX")); break; }
        case 37: { strcpy_P(buffer, PSTR("HLD_LON_P")); break; }
        case 38: { strcpy_P(buffer, PSTR("HLD_LON_I")); break; }
        case 39: { strcpy_P(buffer, PSTR("HLD_LON_IMAX")); break; }
        case 40: { strcpy_P(buffer, PSTR("LOITER_LAT_P")); break; }
        case 41: { strcpy_P(buffer, PSTR("LOITER_LAT_I")); break; }
        case 42: { strcpy_P(buffer, PSTR("LOITER_LAT_D")); break; }
        case 43: { strcpy_P(buffer, PSTR("LOITER_LAT_IMAX")); break; }
        case 44: { strcpy_P(buffer, PSTR("LOITER_LON_P")); break; }
        case 45: { strcpy_P(buffer, PSTR("LOITER_LON_I")); break; }
        case 46: { strcpy_P(buffer, PSTR("LOITER_LON_D")); break; }
        case 47: { strcpy_P(buffer, PSTR("LOITER_LON_IMAX")); break; }
        case 48: { strcpy_P(buffer, PSTR("STAB_D")); break; }
//        case 51: { strcpy_P(buffer, PSTR("STAB_D_S")); break; }
//        case 51: { strcpy_P(buffer, PSTR("ACRO_P")); break; }
//        case 53: { strcpy_P(buffer, PSTR("AXIS_P"));break; }
//        case 54: { strcpy_P(buffer, PSTR("AXIS_ENABLE"));break; }
        default: { return -1; }
      }//end switch(index)
    }// end droneType == 2 
    break;
    */
    default: { return -1; }
    break;
  } //end switch(droneType)
  return 0;
}


int find_param(const char* key)
{
  char buffer[16];
  for (int i=0; i<TOTAL_PARAMS; i++)
  {
    get_Param_Key(buffer, i);


    if (strcmp(buffer,(const char*)key) == 0)
      return i;    
  }
  return -1;  
}

void get_params()
{
  if (paramsRecv >= TOTAL_PARAMS-1)
  {
    return;
  }
  if (timeOut == GET_PARAMS_TIMEOUT)// || timeOut == 80 || timeOut == 60 || timeOut == 40 || timeOut == 20)  // request parameter every 2 seconds
  {
//      char buffer[15];
//      get_Param_Key(buffer, paramsRecv);
      mavlink_message_t msgp;
//      mavlink_msg_param_request_read_pack(127, 0, &msgp, 7, 1, (int8_t *)buffer, 0);
      mavlink_msg_param_request_list_pack(127, 0, &msgp, received_sysid, received_compid);
      send_message(&msgp);
      timeOut--; //just so we don't send twice during the same cycle...
  }
  if (millis() - timer > 100)
  {
    timer = millis();
    timeOut--;
  }

}




