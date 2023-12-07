#include "BoxTree.h"

using namespace PlanarPhysics;

//----------------------------------- BoxTree -----------------------------------

BoxTree::BoxTree()
{
	this->rootNode = nullptr;
}

/*virtual*/ BoxTree::~BoxTree()
{
	this->Clear();
}

bool BoxTree::Insert(Member* member)
{
	if (!this->rootNode)
		this->rootNode = new Node(this->treeBox, nullptr);

	return this->rootNode->Insert(member);
}

void BoxTree::Clear()
{
	delete this->rootNode;
	this->rootNode = nullptr;
}

void BoxTree::ForAllOverlaps(const BoundingBox& box, OverlapFunc overlapFunc)
{
	if (!this->rootNode)
		return;

	return this->rootNode->ForAllOverlaps(box, overlapFunc);
}

void BoxTree::ForAllOverlaps(Member* member, OverlapFunc overlapFunc)
{
	BoundingBox memberBox;
	member->GetBoundingBox(memberBox);

	this->ForAllOverlaps(memberBox, overlapFunc);
}

//----------------------------------- BoxTree::Member -----------------------------------

BoxTree::Member::Member()
{
	this->node = nullptr;
	this->cachedBoundingBoxValid = false;
}

/*virtual*/ BoxTree::Member::~Member()
{
}

bool BoxTree::Member::RemoveFromBoxTree()
{
	if (!this->node)
		return false;

	return this->node->Remove(this);
}

void BoxTree::Member::GetBoundingBox(BoundingBox& box) const
{
	if (!this->cachedBoundingBoxValid)
	{
		this->CalcBoundingBox(this->cachedBoundingBox);
		this->cachedBoundingBoxValid = true;
	}

	box = this->cachedBoundingBox;
}

void BoxTree::Member::UpdateBoxTreeLocation(BoxTree* boxTree)
{
	if (!this->node)
	{
		boxTree->Insert(this);
		return;
	}
	
	BoundingBox memberBox;
	this->GetBoundingBox(memberBox);

	if (!this->node->box.ContainsBox(memberBox))
	{
		Node* newNode = this->node->parent;
		this->node->Remove(const_cast<Member*>(this));
		while (newNode && !newNode->Insert(const_cast<Member*>(this)))
			newNode = newNode->parent;
	}
	else
	{
		for (int i = 0; i < 2; i++)
		{
			Node* childNode = this->node->child[i];
			if (childNode->box.ContainsBox(memberBox))
			{
				this->node->Remove(const_cast<Member*>(this));
				childNode->Insert(this);
			}
		}
	}
}

//----------------------------------- BoxTree::Node -----------------------------------

BoxTree::Node::Node(const BoundingBox& box, Node* parent)
{
	this->memberArray = new std::vector<Member*>();
	this->child[0] = nullptr;
	this->child[1] = nullptr;
	this->parent = parent;
	this->box = box;
}

/*virtual*/ BoxTree::Node::~Node()
{
	delete this->memberArray;
	delete this->child[0];
	delete this->child[1];
}

bool BoxTree::Node::Insert(Member* member)
{
	if (member->node)
		return false;

	BoundingBox memberBox;
	member->GetBoundingBox(memberBox);

	if (!this->box.ContainsBox(memberBox))
		return false;

	if (!(this->child[0] && this->child[1]))
	{
		BoundingBox boxA, boxB;
		this->box.Split(boxA, boxB);

		this->child[0] = new Node(boxA, this);
		this->child[1] = new Node(boxB, this);
	}

	if (!this->child[0]->Insert(member) && !this->child[1]->Insert(member))
	{
		this->memberArray->push_back(member);
		member->node = this;
	}

	return true;
}

bool BoxTree::Node::Remove(Member* member)
{
	for (int i = 0; i < (signed)this->memberArray->size(); i++)
	{
		if ((*this->memberArray)[i] == member)
		{
			if (i < (signed)this->memberArray->size() - 1)
				(*this->memberArray)[i] = (*this->memberArray)[this->memberArray->size() - 1];

			this->memberArray->pop_back();
			member->node = nullptr;
			return true;
		}
	}

	return false;
}

void BoxTree::Node::ForAllOverlaps(const BoundingBox& givenBox, OverlapFunc& overlapFunc)
{
	if (!givenBox.OverlapsWith(this->box))
		return;

	for (Member* member : *this->memberArray)
	{
		BoundingBox memberBox;
		member->GetBoundingBox(memberBox);
		if (givenBox.OverlapsWith(memberBox))
			overlapFunc(member);
	}

	for (int i = 0; i < 2; i++)
		if (this->child[i])
			this->child[i]->ForAllOverlaps(givenBox, overlapFunc);
}