/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcBooleanOperand.h"
#include "IfcCsgSelect.h"
#include "IfcGeometricRepresentationItem.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcBooleanOperator;
	class IFCQUERY_EXPORT IfcBooleanOperand;
	//ENTITY
	class IFCQUERY_EXPORT IfcBooleanResult : virtual public IfcBooleanOperand, virtual public IfcCsgSelect, public IfcGeometricRepresentationItem
	{
	public:
		IfcBooleanResult() = default;
		IfcBooleanResult( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 2736907675; }

		// IfcRepresentationItem -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignment_inverse;
		//  std::vector<weak_ptr<IfcStyledItem> >					m_StyledByItem_inverse;

		// IfcGeometricRepresentationItem -----------------------------------------------------------

		// IfcBooleanResult -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcBooleanOperator>							m_Operator;
		shared_ptr<IfcBooleanOperand>							m_FirstOperand;
		shared_ptr<IfcBooleanOperand>							m_SecondOperand;
	};
}
