#include "wmp_interface.h"

int main(int argc, char *argv[]){
   Msg m;

   if (argc < 3){
      printf("Usage: %s interface ioctl_cmd params\n", argv[0]);
      return 0;
   }

   if(! wmpInit(argv[1])){
      printf("Error iniciando RT-WMP\n");
      return 0;
   }

   m.src = wmpGetNodeId();
   m.priority = 0;
   m.port = 0;

   if (strcmp(argv[2], "push") == 0) {
      int ret;
      printf("Push\n");

      m.dest = atoi(argv[3]);
      m.len = strlen(argv[4])+1;
      strncpy(m.data, argv[4], m.len);


      if((ret = wmpPush(&m))){
         printf("Len: %d Data: \"%s\"\n",m.len ,m.data);
      } else{
         printf("Algo ha ido mal (ret =%d).\n",ret);
      }
   } else if (strcmp(argv[2], "pop") == 0) {
      printf("Pop\n");
      wmpPop(&m);

      printf("From: %d Data: \"%s\"\n",m.src, m.data);
   } else if (strcmp(argv[2], "timedpop") == 0) {
      printf("Timed pop\n");
      if (wmpTimedPop(&m, atoi(argv[3])) == -1){
         printf("Timeout expired\n");
      } else {
         printf("From: %d Data: \"%s\"\n",m.src, m.data);
      }
   } else if (strcmp(argv[2], "nonblockingpop") == 0) {
      printf("Non blocking pop\n");
      if (wmpNonBlockingPop(&m) == -1){
         printf("No message to pop.\n");
      } else{
         printf("From: %d Data: \"%s\"\n",m.src, m.data);
      }
   } else if (strcmp(argv[2], "nodeid") == 0) {
      printf("Node id: %d\n", wmpGetNodeId());
   } else if (strcmp(argv[2], "numofnodes") == 0) {
      printf("Num of nodes: %d\n", wmpGetNumOfNodes());
   } else if (strcmp(argv[2], "getlatestlqm") == 0) {
      int i, j, size = wmpGetNumOfNodes();
      char lqm[size*size];
      printf("LQM:\n");
      wmpGetLatestLQM(lqm);

      for (i = 0; i<size ; i++){
         for (j = 0; j<size ; j++){
            printf(" %04d ",lqm[i*size+j]);
         }
         printf("\n");
      }
   } else if (strcmp(argv[2], "isnetworkconnected") == 0) {
      printf("isNetworkConnected: ");
      if (wmpIsNetworkConnected()){
         printf("CONNECTED\n");
      }  else{
         printf("NOT CONNECTED\n");
      }
   } else if (strcmp(argv[2], "isnetworkconnectedblocking") == 0) {
      printf("isNetworkConnectedBlocking: ");
      if (wmpIsNetworkConnectedBlocking(atoi(argv[3]))){
         printf("CONNECTED\n");
      }      else{
         printf("NOT CONNECTED\n");
      }
   }
   else if (strcmp(argv[2], "removetxmsg") == 0) {
      printf("RemoveTxMsg\n");
      wmp_queue_tx_remove_head();
   }
   else if (strcmp(argv[2], "setcpudelay") == 0) {
      printf("Set CPU delay: %s\n", argv[3]);
      wmpSetCpuDelay(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "getcpudelay") == 0) {
      printf("CPU delay = %d\n", wmpGetCpuDelay());
   }
   else if (strcmp(argv[2], "settimeout") == 0) {
      printf("Set timeout: %s\n", argv[3]);
      wmpSetTimeout(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "gettimeout") == 0) {
      printf("Timeout = %d\n", wmpGetTimeout());
   }
   else if (strcmp(argv[2], "setwcmult") == 0) {
      printf("Set WC mult: %s\n", argv[3]);
      wmpSetWCMult(atof(argv[3]));
   }
   else if (strcmp(argv[2], "getwcmult") == 0) {
      printf("WC mult = %f\n", wmpGetWCMult());
   }
   else if (strcmp(argv[2], "setrate") == 0) {
      printf("Set rate: %s\n", argv[3]);
      wmpSetRate(atof(argv[3]));
   }
   else if (strcmp(argv[2], "getrate") == 0) {
      printf("Rate = %f\n", wmpGetRate());
   }

   else if (strcmp(argv[2], "getnumoffreepositionsintxqueue") == 0) {
      printf("Free positions in TX queue: %d\n", wmp_queue_tx_get_room());
   }
   else if (strcmp(argv[2], "getnumofelementsintxqueue") == 0) {
      printf("Elements in TX queue: %d\n", wmpGetNumOfElementsInTXQueue());
   }
   else if (strcmp(argv[2], "getnumofelementsinrxqueue") == 0) {
      printf("Elements in RX queue: %d\n", wmpGetNumOfElementsInRXQueue());
   }

   else if (strcmp(argv[2], "getnetit") == 0) {
      printf("Net IT: %d\n", wmpGetNetIT());
   }
   else if (strcmp(argv[2], "getmtu") == 0) {
      printf("MTU: %d\n", wmpGetMTU());
   }

   else if (strcmp(argv[2], "setactivesearch") == 0) {
      printf("Set active search\n");
      wmpSetActiveSearch(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "getactivesearch") == 0) {
      printf("Active Search: %d\n", wmpGetActiveSearch());
   }

   else if (strcmp(argv[2], "setinstanceid") == 0) {
      printf("Set instance ID\n");
      wmpSetInstanceId(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "setprimbasedrouting") == 0) {
      printf("Set Prim based routing\n");
      wmpSetPrimBasedRouting(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "setmessagereschedule") == 0) {
      printf("Set message reschedule\n");
      wmpSetMessageReschedule(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "setflowcontrol") == 0) {
      printf("Set flow control\n");
      wmpSetFlowControl(atoi(argv[3]));
   }
   else if (strcmp(argv[2], "getinstanceid") == 0) {
      printf("Instance ID = %d\n",wmpGetInstanceId());
   }
   else if (strcmp(argv[2], "getprimbasedrouting") == 0) {
      printf("Prim based routing = %d\n", wmpGetPrimBasedRouting());
   }
   else if (strcmp(argv[2], "getmessagereschedule") == 0) {
      printf("Message reschedule = %d\n", wmpGetMessageReschedule());
   }
   else if (strcmp(argv[2], "getflowcontrol") == 0) {
      printf("Flow control = %d\n", wmpGetFlowControl());
   }

   else if (strcmp(argv[2], "getserial") == 0) {
      printf("Serial: %u\n", wmpGetSerial());
   }
   else if (strcmp(argv[2], "getloopid") == 0) {
      printf("Loop ID: %u\n", wmpGetLoopId());
   }




   else if (strcmp(argv[2], "addplugin") == 0) {
      if (strcmp(argv[3], "qos") == 0){
         wmpAddPlugin(QOS);
      }
      else if (strcmp(argv[3], "bc_plus") == 0){
         wmpAddPlugin(BC_PLUS);
      }
      else{
         printf("Plugin \"%s\" does not exist.\n", argv[3]);
      }
   }

   else if (strcmp(argv[2], "pushbc") == 0) {
      printf("PushBC\n");

      m.dest = (1 << (wmpGetNumOfNodes())) - 1;
      m.len = strlen(argv[3])+1;
      strncpy(m.data, argv[3],m.len);

      printf("Len: %d Data: \"%s\"\n",m.len ,m.data);

      wmpPushBC(&m);
   }
   else if (strcmp(argv[2], "popbc") == 0) {
      printf("PopBC\n");
      wmpPopBC(&m);

      printf("From: %d Data: \"%s\"\n",m.src, m.data);
   }
   else if (strcmp(argv[2], "timedpopbc") == 0) {
      int timeout = atoi(argv[3]);

      printf("Timed pop BC, timeout = %d ms\n", timeout);
      if (wmpPopBCTimed(&m, timeout) == -1){
         printf("Timeout expired\n");
      }
      else{
         printf("From: %d Data: \"%s\"\n",m.src, m.data);
      }
   }
   else if (strcmp(argv[2], "getnumoffreepositionsintxbcqueue") == 0) {
      printf("Free positions in TXBC queue: %d\n", wmpGetNumOfFreePositionsInTXBCQueue());
   }
   else if (strcmp(argv[2], "getnumofelementsintxbcqueue") == 0) {
      printf("Elements in TXBC queue: %d\n", wmpGetNumOfElementsInTXBCQueue());
   }
   else if (strcmp(argv[2], "getnumofelementsinrxbcqueue") == 0) {
      printf("Elements in RXBC queue: %d\n", wmpGetNumOfElementsInRXBCQueue());
   }

   else if (strcmp(argv[2], "pushqos") == 0) {
      printf("PushQoS\n");

      m.dest = atoi(argv[3]);
      m.len = strlen(argv[4])+1;
      strncpy(m.data, argv[4], m.len);
      m.qos.deadline = 150;

      printf("Len: %d Data: \"%s\"\n",m.len ,m.data);

      wmpPushQoS(&m);
   }
   else if (strcmp(argv[2], "popqos") == 0) {
      printf("PopQoS\n");
      wmpPopQoS(&m);

      printf("From: %d Data: \"%s\"\n",m.src, m.data);
   }
   else if (strcmp(argv[2], "timedpopqos") == 0) {
      int timeout = atoi(argv[3]);

      printf("Timed pop QoS, timeout = %d ms\n", timeout);
      if (wmpPopQoSTimed(&m, timeout) == -1){
         printf("Timeout expired\n");
      }
      else{
         printf("From: %d Data: \"%s\"\n",m.src, m.data);
      }
   }
   else if (strcmp(argv[2], "getnumoffreepositionsintxqosqueue") == 0) {
      printf("Free positions in TXQoS queue: %d\n", qos_tx_getNumOfFreePositionsInQueue());
   }

   else {
      printf("Command \"%s\" not implemented.\n", argv[2]);
   }

   wmpClose();

   return 0;
}
