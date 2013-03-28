#ifdef __KERNEL__
#include <linux/rtnetlink.h>
#include <linux/ctype.h>

MODULE_LICENSE("GPL");

/* Module parameter */
char *autonomous = "no";
module_param(autonomous, charp, 0);
MODULE_PARM_DESC(autonomous, "Indicates if the protocol will use an interface or will be autonomous");

uint node_id = 0;
module_param(node_id, uint, 0);
MODULE_PARM_DESC(node_id, "ID of the node.");

uint num_nodes = 0;
module_param(num_nodes, uint, 0);
MODULE_PARM_DESC(num_nodes, "Number of nodes in the network.");


#define TOUPPER(p) {unsigned int i;\
                   for(i=0; p[i]; i++){\
                      p[i]=toupper(p[i]);\
                   }}

static int run_auto = 0;

static int __init load_rtwmp(void)
{
	printk(KERN_INFO "Module RT-WMP loading\n");
	if(strcmp(autonomous, "yes") == 0) {
		printk(KERN_INFO "Running mode: Autonomous\n");
		run_auto = 1;
		rtnl_lock();
		if(!wmpSetup(node_id, num_nodes)) {
			printk(KERN_ERR "RT-WMP initialization error.\n");
			return -EAGAIN;
		}
		rtnl_unlock();
		wmpRunBG();
	}
	else {
		printk(KERN_INFO "Running mode: With interface\n");
	}
	return 0;
}

static void __exit unload_rtwmp(void)
{
	if(run_auto) {
		wmpExit();
	}
	printk(KERN_INFO "Module RT-WMP unloaded\n");
}
int init_proc(void);
void close_proc(void);

#define INIT_PROC() init_proc()
#define CLOSE_PROC() close_proc()

module_init(load_rtwmp);
module_exit(unload_rtwmp);

EXPORT_SYMBOLS();

#else

#define INIT_PROC() 1
#define CLOSE_PROC()

int  wmpIsKernelSpace(){
	return 0;
}

#endif
