#pragma once

#include "Math/Utilities/BoundingBox.h"

namespace PlanarPhysics
{
	class PLANAR_PHYSICS_API BoxTree
	{
	private:
		class Node;

	public:
		BoxTree();
		virtual ~BoxTree();

		BoundingBox treeBox;

		class Member;

		typedef std::function<void(Member*)> OverlapFunc;

		bool Insert(Member* member);
		void Clear();
		void ForAllOverlaps(const BoundingBox& box, OverlapFunc overlapFunc);
		void ForAllOverlaps(Member* member, OverlapFunc overlapFunc);

		class PLANAR_PHYSICS_API Member
		{
			friend class Node;

		public:
			Member();
			virtual ~Member();

			void GetBoundingBox(BoundingBox& box) const;
			bool RemoveFromBoxTree();
			void UpdateBoxTreeLocation(BoxTree* boxTree);

			virtual void CalcBoundingBox(BoundingBox& box) const = 0;

		protected:
			mutable BoundingBox cachedBoundingBox;
			mutable bool cachedBoundingBoxValid;

		private:
			Node* node;
		};

	private:

		class Node
		{
		public:
			Node(const BoundingBox& box, Node* parent);
			virtual ~Node();

			bool Insert(Member* member);
			bool Remove(Member* member);
			void ForAllOverlaps(const BoundingBox& givenBox, OverlapFunc& overlapFunc);

			std::vector<Member*>* memberArray;
			BoundingBox box;
			Node* child[2];
			Node* parent;
		};

		Node* rootNode;
	};
}