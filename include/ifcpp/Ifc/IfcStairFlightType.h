/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcBuiltElementType.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcStairFlightTypeEnum;
	//ENTITY
	class IFCQUERY_EXPORT IfcStairFlightType : public IfcBuiltElementType
	{
	public:
		IfcStairFlightType() = default;
		IfcStairFlightType( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 10; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1039846685; }

		// IfcRoot -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcGloballyUniqueId>						m_GlobalId;
		//  shared_ptr<IfcOwnerHistory>							m_OwnerHistory;				//optional
		//  shared_ptr<IfcLabel>								m_Name;						//optional
		//  shared_ptr<IfcText>									m_Description;				//optional

		// IfcObjectDefinition -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcRelAssigns> >				m_HasAssignments_inverse;
		//  std::vector<weak_ptr<IfcRelNests> >					m_Nests_inverse;
		//  std::vector<weak_ptr<IfcRelNests> >					m_IsNestedBy_inverse;
		//  std::vector<weak_ptr<IfcRelDeclares> >				m_HasContext_inverse;
		//  std::vector<weak_ptr<IfcRelAggregates> >			m_IsDecomposedBy_inverse;
		//  std::vector<weak_ptr<IfcRelAggregates> >			m_Decomposes_inverse;
		//  std::vector<weak_ptr<IfcRelAssociates> >			m_HasAssociations_inverse;

		// IfcTypeObject -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcIdentifier>							m_ApplicableOccurrence;		//optional
		//  std::vector<shared_ptr<IfcPropertySetDefinition> >	m_HasPropertySets;			//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcRelDefinesByType> >			m_Types_inverse;

		// IfcTypeProduct -----------------------------------------------------------
		// attributes:
		//  std::vector<shared_ptr<IfcRepresentationMap> >		m_RepresentationMaps;		//optional
		//  shared_ptr<IfcLabel>								m_Tag;						//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcRelAssignsToProduct> >		m_ReferencedBy_inverse;

		// IfcElementType -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcLabel>								m_ElementType;				//optional

		// IfcBuiltElementType -----------------------------------------------------------

		// IfcStairFlightType -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcStairFlightTypeEnum>					m_PredefinedType;
	};
}
