
#include "config/compiler.h"
#include "core/include/global.h"
#include "config/compiler.h"
#include "core/include/prim.h"
#include "core/include/wmp_utils.h"
#include "core/include/rssi_average.h"

static char ** tmp_lqm;
static char * tree, *tree_bak;

void print_matrix(char* txt, char **lqm){
	return;
	char q[1024];
	lqmToString(lqm,q,status.N_NODES);
   WMP_ERROR(stderr,"@%s@\n%s#\n",txt,q);
}

void print_vector(char*txt, char *t){
	return;
	int i, n=status.N_NODES;
   WMP_ERROR(stderr,"@%s@\n",txt);
	for (i=0;i<n;i++){
      WMP_ERROR(stderr,"\t%d",i);
	}
   WMP_ERROR(stderr,"\n");
	for (i=0;i<n;i++){
      WMP_ERROR(stderr,"\t%d",t[i]);
	}
   WMP_ERROR(stderr,"#\n");
}

void tree_save_tree(char * cp){
	memcpy(tree_bak,cp,status.N_NODES);
}

void tree_create_lqm_from_tree(char ** lqm, char * tree){
	int i,j;
	for (i=0;i<status.N_NODES;i++){
		for (j=0;j<status.N_NODES;j++){
			lqm[i][j]=0;
		}
	}
	//FPRINTF(stderr,"Cleaned!\n");
	print_vector("kkk",tree);

	for (i=0;i<status.N_NODES;i++){
		if (tree[i]>=0 && tree[i]< status.N_NODES){
			lqm[i][(int)tree[i]]=100;
			lqm[(int)tree[i]][i]=100;
		}
	}
	print_matrix("kkk",lqm);
}
int error_happened=0;
void tree_create_tree_from_lqm1(char * tree, char ** lqm){
	int i,j;

	char occupied[status.N_NODES];
	for (i=0;i<status.N_NODES;i++){
		 tree[i]=-1;
		 occupied[i]=0;
	}
	for (i=0;i<status.N_NODES;i++){
		for (j=0;j<i;j++){
			if (lqm[i][j]>0){
				if (occupied[i]==0){
					tree[i]=j;
					occupied[i]=1;
					if (j==0) tree[j]=i;
				}else{
					if (occupied[j]==0){
						tree[j]=i;
						if (i==0) tree[i]=j;
					}else{
						WMP_DEBUG(stderr,"compression error\n");
						//assert(0);
						error_happened++;
					}
				}
			}
		}
	}
}

void tree_create_tree_from_lqm2(char * tree, char ** lqm){
	int i,j,k,cnt;

	char occupied[status.N_NODES];
	for (i=0;i<status.N_NODES;i++){
		 tree[i]=-1;
		 occupied[i]=0;
	}
	for (i=0;i<status.N_NODES;i++){
		for (j=0;j<status.N_NODES;j++){
			for (k=j, cnt=0 ;k<status.N_NODES;k++){
				/* count the number of elements in a line */
				cnt += (lqm[i][k] > 0 ? 1 : 0);
			}
			if (lqm[i][j]>0 && j!=i){
					if (cnt>1 && tree[j]==i) continue;
					tree[i]=j;
					break;
			}
		}
	}
}

int tree_copy_lqm(char ** src_lqm, char ** dst_lqm){
   int i,j;
   for (i=0;i<status.N_NODES;i++){
      for (j=0;j<status.N_NODES;j++){
         dst_lqm[i][j]=src_lqm[i][j];
      }
   }
}

void tree_create_tree_from_lqm(char * tree, char ** lqm){
	int i,j,k;
	tree_copy_lqm(lqm,tmp_lqm);
	char occupied[status.N_NODES];
	for (i=0;i<status.N_NODES;i++){
		 tree[i]=-1;
		 occupied[i]=0;
	}
	for (k = 0; k < status.N_NODES; k++) {
		for (i = 0; i < status.N_NODES; i++) {
			int idx = 0, val = -1;
			for (j = 0; j < status.N_NODES; j++) {
				if (tmp_lqm[i][j] > 0) {
					idx++;
					val = j;
				}
			}
			if (idx == 1) {
				tree[i] = val;
				tmp_lqm[val][i] = 0;
			}
		}
	}
}

void tree_init(void){
   int i;

	tmp_lqm=(char **) MALLOC(status.N_NODES*sizeof(char*));
	tree=(char *) MALLOC(status.N_NODES*sizeof(char));
	tree_bak = (char *) MALLOC(status.N_NODES*sizeof(char));

	for (i=0;i<status.N_NODES;i++){
		tmp_lqm[i]=(char *) MALLOC(status.N_NODES*sizeof(char));
	}
}

void tree_free(void){
   int i;

   for (i=0;i<status.N_NODES;i++){
      FREE(tmp_lqm[i]);
   }
   FREE(tmp_lqm);
   FREE(tree_bak);
   FREE(tree);
}

void tree_set_next_tree(char * cp){
	memcpy(tree,cp,status.N_NODES);
}
char * tree_next_tree_get_ptr(void){
	return tree;
}

char * tree_get_saved_tree(void){
	return tree_bak;
}

void tree_get_next_tree(char * cp){
	int i;
	tree_create_lqm_from_tree(tmp_lqm,cp);
	print_matrix("before all",tmp_lqm);
	for (i=0;i<status.N_NODES;i++){
		tmp_lqm[status.id][i]=rssi_get_averaged_rssi(i);
		tmp_lqm[i][status.id]=rssi_get_averaged_rssi(i);
	}
	print_matrix("before prim",tmp_lqm);
	char ** improved_lqm=prim(tmp_lqm);
	print_matrix("after prim",improved_lqm);
	tree_create_tree_from_lqm(tree,improved_lqm);
	memcpy(cp,tree,status.N_NODES);
	print_vector("ap",cp);
}

int tree_which_best(char * t1, char * t2) {
	int broken1 = 0, broken2 = 0, i;
	for (i = 0; i < status.N_NODES; i++) {
		if (t1[i] < 0){
			broken1++;
		}
		if (t2[i] < 0){
			broken2++;
		}
	}
	if (broken1 < broken2) {
		return 1;
	} else {
		return 2;
	}
}

int tree_get_tree_path(char ** lqm, int id, char * path) {
	char reached[32];
	int path_idx = 1, path_pnt = 1, sum = 0, i, done;
	memset(reached, 0, sizeof(reached));

	path[0] = id;
	reached[id] = 1;

	while (sum < status.N_NODES) {
		done = 0;
		for (i = 0; i < status.N_NODES; i++) {
			if (lqm[id][i] && !reached[i]) {
				id = i;
				done = 1;
				reached[id] = 1;
				path[path_idx] = id;
				path_idx++;
				path_pnt++;
				break;
			}
		}
		if (!done) {
			path_pnt -= 1;
			int nid = path[path_pnt];
			if (nid != id) {
				path[path_idx] = nid;
				path_idx++;
			}
			id = nid;
		}

		sum = 0;
		for (i = 0; i < status.N_NODES; i++) {
			sum += reached[i];
		}
	}

//	for (i = 0; i < path_idx; i++) {
//		fprintf(stderr, "%d ", path[i]);
//	}
	return path_idx;
}
