/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcSimpleProperty.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcValue;
	class IFCQUERY_EXPORT IfcUnit;
	//ENTITY
	class IFCQUERY_EXPORT IfcPropertyBoundedValue : public IfcSimpleProperty
	{
	public:
		IfcPropertyBoundedValue() = default;
		IfcPropertyBoundedValue( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 6; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 871118103; }

		// IfcPropertyAbstraction -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcExternalReferenceRelationship> >	m_HasExternalReferences_inverse;

		// IfcProperty -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcIdentifier>									m_Name;
		//  shared_ptr<IfcText>											m_Specification;			//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPropertySet> >						m_PartOfPset_inverse;
		//  std::vector<weak_ptr<IfcPropertyDependencyRelationship> >	m_PropertyForDependance_inverse;
		//  std::vector<weak_ptr<IfcPropertyDependencyRelationship> >	m_PropertyDependsOn_inverse;
		//  std::vector<weak_ptr<IfcComplexProperty> >					m_PartOfComplex_inverse;
		//  std::vector<weak_ptr<IfcResourceConstraintRelationship> >	m_HasConstraints_inverse;
		//  std::vector<weak_ptr<IfcResourceApprovalRelationship> >		m_HasApprovals_inverse;

		// IfcSimpleProperty -----------------------------------------------------------

		// IfcPropertyBoundedValue -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcValue>										m_UpperBoundValue;			//optional
		shared_ptr<IfcValue>										m_LowerBoundValue;			//optional
		shared_ptr<IfcUnit>											m_Unit;						//optional
		shared_ptr<IfcValue>										m_SetPointValue;			//optional
	};
}
