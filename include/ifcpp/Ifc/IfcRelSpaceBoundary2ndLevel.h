/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcRelSpaceBoundary1stLevel.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcRelSpaceBoundary2ndLevel;
	//ENTITY
	class IFCQUERY_EXPORT IfcRelSpaceBoundary2ndLevel : public IfcRelSpaceBoundary1stLevel
	{
	public:
		IfcRelSpaceBoundary2ndLevel() = default;
		IfcRelSpaceBoundary2ndLevel( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 11; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1521410863; }

		// IfcRoot -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcGloballyUniqueId>						m_GlobalId;
		//  shared_ptr<IfcOwnerHistory>							m_OwnerHistory;				//optional
		//  shared_ptr<IfcLabel>								m_Name;						//optional
		//  shared_ptr<IfcText>									m_Description;				//optional

		// IfcRelationship -----------------------------------------------------------

		// IfcRelConnects -----------------------------------------------------------

		// IfcRelSpaceBoundary -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcSpaceBoundarySelect>					m_RelatingSpace;
		//  shared_ptr<IfcElement>								m_RelatedBuildingElement;
		//  shared_ptr<IfcConnectionGeometry>					m_ConnectionGeometry;		//optional
		//  shared_ptr<IfcPhysicalOrVirtualEnum>				m_PhysicalOrVirtualBoundary;
		//  shared_ptr<IfcInternalOrExternalEnum>				m_InternalOrExternalBoundary;

		// IfcRelSpaceBoundary1stLevel -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcRelSpaceBoundary1stLevel>				m_ParentBoundary;			//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcRelSpaceBoundary1stLevel> >	m_InnerBoundaries_inverse;

		// IfcRelSpaceBoundary2ndLevel -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcRelSpaceBoundary2ndLevel>				m_CorrespondingBoundary;	//optional
		// inverse attributes:
		std::vector<weak_ptr<IfcRelSpaceBoundary2ndLevel> >	m_Corresponds_inverse;
	};
}
