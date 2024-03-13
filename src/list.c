//
// Created by hujianzhe
//

#include "../inc/list.h"

#ifdef	__cplusplus
extern "C" {
#endif

struct List_t* listInit(struct List_t* list) {
	list->head = list->tail = (struct ListNode_t*)0;
	return list;
}

void listInsertNodeFront(struct List_t* list, struct ListNode_t* node, struct ListNode_t* new_node) {
	if (!list->head) {
		list->head = list->tail = new_node;
		new_node->prev = new_node->next = (struct ListNode_t*)0;
		return;
	}
	if (list->head == node) {
		list->head = new_node;
	}

	new_node->next = node;
	new_node->prev = node->prev;
	if (node->prev) {
		node->prev->next = new_node;
	}
	node->prev = new_node;
}

void listInsertNodeBack(struct List_t* list, struct ListNode_t* node, struct ListNode_t* new_node) {
	if (!list->tail) {
		list->head = list->tail = new_node;
		new_node->prev = new_node->next = (struct ListNode_t*)0;
		return;
	}
	if (list->tail == node) {
		list->tail = new_node;
	}

	new_node->prev = node;
	new_node->next = node->next;
	if (node->next) {
		node->next->prev = new_node;
	}
	node->next = new_node;
}

void listRemoveNode(struct List_t* list, struct ListNode_t* node) {
	if (list->head == node) {
		list->head = node->next;
	}
	if (list->tail == node) {
		list->tail = node->prev;
	}

	if (node->prev) {
		node->prev->next = node->next;
	}
	if (node->next) {
		node->next->prev = node->prev;
	}
}

void listReplaceNode(struct List_t* list, struct ListNode_t* node, struct ListNode_t* new_node) {
	if (list->head == node) {
		list->head = new_node;
	}
	if (list->tail == node) {
		list->tail = new_node;
	}
	if (!node) {
		new_node->prev = new_node->next = (struct ListNode_t*)0;
		return;
	}

	if (node->prev) {
		node->prev->next = new_node;
	}
	if (node->next) {
		node->next->prev = new_node;
	}
	*new_node = *node;
}

struct List_t* listPushNodeFront(struct List_t* list, struct ListNode_t* node) {
	listInsertNodeFront(list, list->head, node);
	return list;
}

struct List_t* listPushNodeBack(struct List_t* list, struct ListNode_t* node) {
	listInsertNodeBack(list, list->tail, node);
	return list;
}

struct ListNode_t* listPopNodeFront(struct List_t* list) {
	struct ListNode_t* head = list->head;
	if (head)
		listRemoveNode(list, head);
	return head;
}

struct ListNode_t* listPopNodeBack(struct List_t* list) {
	struct ListNode_t* tail = list->tail;
	if (tail)
		listRemoveNode(list, tail);
	return tail;
}

int listIsEmpty(const struct List_t* list) { return !list->head; }

#ifdef	__cplusplus
}
#endif
