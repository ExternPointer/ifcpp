/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcMobileTelecommunicationsApplianceTypeEnum = ENUMERATION OF	(ACCESSPOINT	,BASEBANDUNIT	,BASETRANSCEIVERSTATION	,E_UTRAN_NODE_B	,GATEWAY_GPRS_SUPPORT_NODE	,MASTERUNIT	,MOBILESWITCHINGCENTER	,MSCSERVER	,PACKETCONTROLUNIT	,REMOTERADIOUNIT	,REMOTEUNIT	,SERVICE_GPRS_SUPPORT_NODE	,SUBSCRIBERSERVER	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcMobileTelecommunicationsApplianceTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcMobileTelecommunicationsApplianceTypeEnumEnum
		{
			ENUM_ACCESSPOINT,
			ENUM_BASEBANDUNIT,
			ENUM_BASETRANSCEIVERSTATION,
			ENUM_E_UTRAN_NODE_B,
			ENUM_GATEWAY_GPRS_SUPPORT_NODE,
			ENUM_MASTERUNIT,
			ENUM_MOBILESWITCHINGCENTER,
			ENUM_MSCSERVER,
			ENUM_PACKETCONTROLUNIT,
			ENUM_REMOTERADIOUNIT,
			ENUM_REMOTEUNIT,
			ENUM_SERVICE_GPRS_SUPPORT_NODE,
			ENUM_SUBSCRIBERSERVER,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcMobileTelecommunicationsApplianceTypeEnum() = default;
		IfcMobileTelecommunicationsApplianceTypeEnum( IfcMobileTelecommunicationsApplianceTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 423474865; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcMobileTelecommunicationsApplianceTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcMobileTelecommunicationsApplianceTypeEnumEnum m_enum;
	};
}
