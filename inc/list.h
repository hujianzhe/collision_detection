//
// Created by hujianzhe
//

#ifndef UTIL_C_DATASTRUCT_LIST_H
#define	UTIL_C_DATASTRUCT_LIST_H

typedef struct ListNode_t {
	struct ListNode_t *prev, *next;
} ListNode_t;
typedef struct List_t {
	struct ListNode_t *head, *tail;
} List_t;

#ifdef	__cplusplus
extern "C" {
#endif

struct List_t* listInit(struct List_t* list);
void listInsertNodeFront(struct List_t* list, struct ListNode_t* node, struct ListNode_t* new_node);
void listInsertNodeBack(struct List_t* list, struct ListNode_t* node, struct ListNode_t* new_node);
void listRemoveNode(struct List_t* list, struct ListNode_t* node);
void listReplaceNode(struct List_t* list, struct ListNode_t* node, struct ListNode_t* new_node);

struct List_t* listPushNodeFront(struct List_t* list, struct ListNode_t* node);
struct List_t* listPushNodeBack(struct List_t* list, struct ListNode_t* node);
struct ListNode_t* listPopNodeFront(struct List_t* list);
struct ListNode_t* listPopNodeBack(struct List_t* list);
int listIsEmpty(const struct List_t* list);

#ifdef	__cplusplus
}
#endif

#endif
