/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *
 *
 *
 *  File: ./src/core/proc.c
 *  Authors: Rubén Durán
 *           Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include "config/compiler.h"
#include "core/interface/wmp_interface.h"

/* Variables for the directory and all the files */
static struct proc_dir_entry *directory;

static struct proc_dir_entry *fNodeId;
static struct proc_dir_entry *fNumOfNodes;
static struct proc_dir_entry *fLatestLQM;

static struct proc_dir_entry *fNetworkConnected;

static struct proc_dir_entry *fCpuDelay;
static struct proc_dir_entry *fTimeout;
static struct proc_dir_entry *fWCMult;
static struct proc_dir_entry *fRate;

static struct proc_dir_entry *fFreePositionsInTXQueue;
static struct proc_dir_entry *fElementsInTXQueue;
static struct proc_dir_entry *fElementsInRXQueue;

static struct proc_dir_entry *fNetIT;
static struct proc_dir_entry *fMTU;

static struct proc_dir_entry *fActiveSearch;

static struct proc_dir_entry *fInstanceId;
static struct proc_dir_entry *fPrimBasedRouting;
static struct proc_dir_entry *fMessageReschedule;
static struct proc_dir_entry *fFlowControl;

static struct proc_dir_entry *fSerial;
static struct proc_dir_entry *fLoopId;



/* Functions for reading and writing the files */
int f_getNodeId(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetNodeId());
   }
}

int f_getNumOfNodes(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetNumOfNodes());
   }
}

int f_getLatestLQM(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      int i, j, size = wmpGetNumOfNodes();
      char lqm[size*size];
      char *p = page;
      wmpGetLatestLQM(lqm);

      for (i = 0; i<size ; i++){
         for (j = 0; j<size ; j++){
            p += sprintf(p, " %04d ",lqm[i*size+j]);
         }
         p += sprintf(p, "\n");
      }
      return p-page;             // Returns the size
   }
}

int f_networkConnected(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      if(wmpIsNetworkConnected()){
         return sprintf(page, "Connected\n");
      }
      else{
         return sprintf(page, "Not connected\n");
      }
   }
}

int f_CpuDelay_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetCpuDelay());
   }
}
int f_CpuDelay_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[20];
   int size = (len>20?20:len);
   int val;

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';

   val = atoi(aux);

   if(val <= 0){
      printk(KERN_INFO "Delay must be over 0.");
   }
   else{
      wmpSetCpuDelay(val);
   }

   return len;
}

int f_Timeout_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetTimeout());
   }
}
int f_Timeout_write(struct file *f, const char __user *buff, unsigned long len, void *data ) {
   char aux[20];
   int size = (len>20?20:len);
   int val;

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';

   val = atoi(aux);

   if(val < 0){
      printk(KERN_INFO "Timeout must be at least 0.");
   }
   else{
      wmpSetTimeout(val);
   }

   return len;
}

int f_WCMult_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      int mult = wmpGetWCMult();
      return sprintf(page, "%d\n", (int) mult);
   }
}
int f_WCMult_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[32];
   int size = (len>32?32:len);
   int val;

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';

   val = atoi(aux);
   wmpSetWCMult(val);

   return len;
}

int f_Rate_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      unsigned long decimales;
      int rate = wmpGetRate();
     // kernel_fpu_begin();
      decimales = (rate- (int)rate)*1000;
     // kernel_fpu_end();
      return sprintf(page, "%d.%03lu\n", (int)rate, decimales);
   }
}
int f_Rate_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[32];
   int size = (len>32?32:len);
   int val;

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';

   val = atoi(aux);
   wmpSetRate(val);

   return len;
}

int f_getFreePositionsInTXQueue(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmp_queue_tx_get_room());
   }
}

int f_getElementsInTXQueue(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetNumOfElementsInTXQueue());
   }
}

int f_getElementsInRXQueue(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetNumOfElementsInRXQueue(0));
   }
}

int f_getNetIT(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetNetIT());
   }
}

int f_getMTU(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%u\n", wmpGetMTU());
   }
}

int f_activeSearch_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[10];
   int val, size;
   size = (len>10?10:len);

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';
   val = atoi(aux);
   if(val != 0 && val != 1){
      printk(KERN_INFO "The only possible values are 0 (off) and 1 (on).\n");
   }
   else{
      wmpSetActiveSearch(val);
   }

   return len;
}
int f_activeSearch_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetActiveSearch());
   }
}

int f_instanceId_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[10];
   int val, size;
   size = (len>10?10:len);

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';
   val = atoi(aux);
   if(val <= 0){
      printk(KERN_INFO "The instance ID must be positive.\n");
   }
   else{
      wmpSetInstanceId(val);
   }

   return len;
}
int f_instanceId_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetInstanceId());
   }
}

int f_primBasedRouting_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[10];
   int val, size;
   size = (len>10?10:len);

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';
   val = atoi(aux);

   wmpSetPrimBasedRouting(val);

   return len;
}
int f_primBasedRouting_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetPrimBasedRouting());
   }
}

int f_messageReschedule_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[10];
   int val, size;
   size = (len>10?10:len);

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';
   val = atoi(aux);

   wmpSetMessageReschedule(val);

   return len;
}
int f_messageReschedule_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetMessageReschedule());
   }
}

int f_flowControl_write(struct file *f, const char __user *buff, unsigned long len, void *data ){
   char aux[10];
   int val, size;
   size = (len>10?10:len);

   if (copy_from_user(aux, buff, size)) {
      return -EFAULT;
   }
   aux[size-1]='\0';
   val = atoi(aux);

   wmpSetFlowControl(val);

   return len;
}
int f_flowControl_read(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%d\n", wmpGetFlowControl());
   }
}

int f_serial(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%u\n", wmpGetSerial());
   }
}

int f_loopId(char *page, char **start, off_t off, int count, int *eof, void *data){
   if (off > 0) {
      *eof = 1;
      return 0;
   }
   else {
      return sprintf(page, "%u\n", wmpGetLoopId());
   }
}

typedef enum { NODEID, NUMOFNODES, LATESTLQM, NETWORKCONNECTED, CPUDELAY,
               TIMEOUT, WCMULT, RATE, FREEPOSTX, ELEMSTX, ELEMSRX, NETIT, GETMTU,
               ACTIVESEARCH, INSTANCEID, PRIMBASEDROUTING, MESSAGERESCHEDULE,
               FLOWCONTROL, SERIAL, LOOPID, ALL} tpClose;

/* Removes /proc/rt-wmp/ and all the files inside it created before 'where' */
void __close_proc(tpClose where){
   switch(where){
      case ALL:
         remove_proc_entry(fLoopId->name, fLoopId->parent);
      case LOOPID:
         remove_proc_entry(fSerial->name, fSerial->parent);
      case SERIAL:
         remove_proc_entry(fFlowControl->name, fFlowControl->parent);
      case FLOWCONTROL:
         remove_proc_entry(fMessageReschedule->name, fMessageReschedule->parent);
      case MESSAGERESCHEDULE:
         remove_proc_entry(fPrimBasedRouting->name, fPrimBasedRouting->parent);
      case PRIMBASEDROUTING:
         remove_proc_entry(fInstanceId->name, fInstanceId->parent);
      case INSTANCEID:
         remove_proc_entry(fActiveSearch->name, fActiveSearch->parent);
      case ACTIVESEARCH:
         remove_proc_entry(fMTU->name, fMTU->parent);
      case GETMTU:
         remove_proc_entry(fNetIT->name, fNetIT->parent);
      case NETIT:
         remove_proc_entry(fElementsInRXQueue->name, fElementsInRXQueue->parent);
      case ELEMSRX:
         remove_proc_entry(fElementsInTXQueue->name, fElementsInTXQueue->parent);
      case ELEMSTX:
         remove_proc_entry(fFreePositionsInTXQueue->name, fFreePositionsInTXQueue->parent);
      case FREEPOSTX:
         remove_proc_entry(fRate->name, fRate->parent);
      case RATE:
         remove_proc_entry(fWCMult->name, fWCMult->parent);
      case WCMULT:
         remove_proc_entry(fTimeout->name, fTimeout->parent);
      case TIMEOUT:
         remove_proc_entry(fCpuDelay->name, fCpuDelay->parent);
      case CPUDELAY:
         remove_proc_entry(fNetworkConnected->name, fNetworkConnected->parent);
      case NETWORKCONNECTED:
         remove_proc_entry(fLatestLQM->name, fLatestLQM->parent);
      case LATESTLQM:
         remove_proc_entry(fNumOfNodes->name, fNumOfNodes->parent);
      case NUMOFNODES:
         remove_proc_entry(fNodeId->name, fNodeId->parent);
      case NODEID:
         remove_proc_entry(directory->name, directory->parent);
      //case DIR:
   }
}

/* Creates /proc/rt-wmp/ and all the files inside it */
int init_proc(void){
   directory = proc_mkdir( "rt-wmp",NULL );
   if(!directory){
      //__close_proc(DIR);
      return -ENOMEM;
   }

   fNodeId = create_proc_read_entry("nodeId", 0666, directory, f_getNodeId, NULL);
   if(!fNodeId){
      __close_proc(NODEID);
      return -ENOMEM;
   }

   fNumOfNodes = create_proc_read_entry("numOfNodes", 0666, directory, f_getNumOfNodes, NULL);
   if(!fNumOfNodes){
      __close_proc(NUMOFNODES);
      return -ENOMEM;
   }

   fLatestLQM = create_proc_read_entry("latestLQM", 0666, directory, f_getLatestLQM, NULL);
   if(!fLatestLQM){
      __close_proc(LATESTLQM);
      return -ENOMEM;
   }

   fNetworkConnected = create_proc_read_entry("networkConnected", 0666, directory, f_networkConnected, NULL);
   if(!fNetworkConnected){
      __close_proc(NETWORKCONNECTED);
      return -ENOMEM;
   }

   fCpuDelay = create_proc_entry("cpuDelay", 0666, directory);
   if(!fCpuDelay){
      __close_proc(CPUDELAY);
      return -ENOMEM;
   }
   fCpuDelay->read_proc = f_CpuDelay_read;
   fCpuDelay->write_proc = f_CpuDelay_write;

   fTimeout = create_proc_entry("timeout", 0666, directory);
   if(!fTimeout){
      __close_proc(TIMEOUT);
      return -ENOMEM;
   }
   fTimeout->read_proc = f_Timeout_read;
   fTimeout->write_proc = f_Timeout_write;

   fWCMult = create_proc_entry("WCMult", 0666, directory);
   if(!fWCMult){
      __close_proc(WCMULT);
      return -ENOMEM;
   }
   fWCMult->read_proc = f_WCMult_read;
   fWCMult->write_proc = f_WCMult_write;

   fRate = create_proc_entry("rate", 0666, directory);
   if(!fRate){
      __close_proc(RATE);
      return -ENOMEM;
   }
   fRate->read_proc = f_Rate_read;
   fRate->write_proc = f_Rate_write;

   fFreePositionsInTXQueue = create_proc_read_entry("freePositionsInTXQueue", 0666, directory, f_getFreePositionsInTXQueue, NULL);
   if(!fFreePositionsInTXQueue){
      __close_proc(FREEPOSTX);
      return -ENOMEM;
   }

   fElementsInTXQueue = create_proc_read_entry("elementsInTXQueue", 0666, directory, f_getElementsInTXQueue, NULL);
   if(!fElementsInTXQueue){
      __close_proc(ELEMSTX);
      return -ENOMEM;
   }

   fElementsInRXQueue = create_proc_read_entry("elementsInRXQueue", 0666, directory, f_getElementsInRXQueue, NULL);
   if(!fElementsInRXQueue){
      __close_proc(ELEMSRX);
      return -ENOMEM;
   }

   fNetIT = create_proc_read_entry("netIT", 0666, directory, f_getNetIT, NULL);
   if(!fNetIT){
      __close_proc(NETIT);
      return -ENOMEM;
   }

   fMTU = create_proc_read_entry("mtu", 0666, directory, f_getMTU, NULL);
   if(!fMTU){
      __close_proc(GETMTU);
      return -ENOMEM;
   }

   fActiveSearch = create_proc_entry("activeSearch", 0666, directory);
   if(!fActiveSearch){
      __close_proc(ACTIVESEARCH);
      return -ENOMEM;
   }
   fActiveSearch->read_proc = f_activeSearch_read;
   fActiveSearch->write_proc = f_activeSearch_write;

   fInstanceId = create_proc_entry("instanceId", 0666, directory);
   if(!fInstanceId){
      __close_proc(INSTANCEID);
      return -ENOMEM;
   }
   fInstanceId->read_proc = f_instanceId_read;
   fInstanceId->write_proc = f_instanceId_write;

   fPrimBasedRouting = create_proc_entry("primBasedRouting", 0666, directory);
   if(!fPrimBasedRouting){
      __close_proc(PRIMBASEDROUTING);
      return -ENOMEM;
   }
   fPrimBasedRouting->read_proc = f_primBasedRouting_read;
   fPrimBasedRouting->write_proc = f_primBasedRouting_write;

   fMessageReschedule = create_proc_entry("messageReschedule", 0666, directory);
   if(!fMessageReschedule){
      __close_proc(MESSAGERESCHEDULE);
      return -ENOMEM;
   }
   fMessageReschedule->read_proc = f_messageReschedule_read;
   fMessageReschedule->write_proc = f_messageReschedule_write;

   fFlowControl = create_proc_entry("flowControl", 0666, directory);
   if(!fFlowControl){
      __close_proc(FLOWCONTROL);
      return -ENOMEM;
   }
   fFlowControl->read_proc = f_flowControl_read;
   fFlowControl->write_proc = f_flowControl_write;

   fSerial = create_proc_read_entry("serial", 0666, directory, f_serial, NULL);
   if(!fSerial){
      __close_proc(SERIAL);
      return -ENOMEM;
   }

   fLoopId = create_proc_read_entry("loopId", 0666, directory, f_loopId, NULL);
   if(!fLoopId){
      __close_proc(LOOPID);
      return -ENOMEM;
   }

   return 1;
}

/* Removes /proc/rt-wmp/ and all the files inside it */
void close_proc(void){
   __close_proc(ALL);
}
