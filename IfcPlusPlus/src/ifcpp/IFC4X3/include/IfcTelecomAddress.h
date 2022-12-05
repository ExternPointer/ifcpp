/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"
#include "IfcAddress.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcLabel;
	class IFCQUERY_EXPORT IfcURIReference;
	//ENTITY
	class IFCQUERY_EXPORT IfcTelecomAddress : public IfcAddress
	{ 
	public:
		IfcTelecomAddress() = default;
		IfcTelecomAddress( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 9; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 912023232; }

		// IfcAddress -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcAddressTypeEnum>				m_Purpose;					//optional
		//  shared_ptr<IfcText>							m_Description;				//optional
		//  shared_ptr<IfcLabel>						m_UserDefinedPurpose;		//optional
		// inverse attributes:
		//  std::vector<weak_ptr<IfcPerson> >			m_OfPerson_inverse;
		//  std::vector<weak_ptr<IfcOrganization> >		m_OfOrganization_inverse;

		// IfcTelecomAddress -----------------------------------------------------------
		// attributes:
		std::vector<shared_ptr<IfcLabel> >			m_TelephoneNumbers;			//optional
		std::vector<shared_ptr<IfcLabel> >			m_FacsimileNumbers;			//optional
		shared_ptr<IfcLabel>						m_PagerNumber;				//optional
		std::vector<shared_ptr<IfcLabel> >			m_ElectronicMailAddresses;	//optional
		shared_ptr<IfcURIReference>					m_WWWHomePageURL;			//optional
		std::vector<shared_ptr<IfcURIReference> >	m_MessagingIDs;				//optional
	};
}

