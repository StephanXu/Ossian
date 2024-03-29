#ifndef OSSIAN_CLIENT_GRAPHIC
#define OSSIAN_CLIENT_GRAPHIC

#include <ossian/Factory.hpp>

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <array>
#include <cstring>
#include <queue>

#include "Referee.hpp"

//[ATTENTION]: We didn't check any range of arguments

class IClientGraphicElementStyle
{
public:
	virtual auto FillGraphicData(GraphicData& graphicData) const -> void = 0;
};

class LineStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;
	uint16_t m_EndX;
	uint16_t m_EndY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType          = 0;
		graphicData.m_Color                = m_Color;
		graphicData.m_Width                = m_Width;
		graphicData.m_StartX               = m_StartX;
		graphicData.m_StartY               = m_StartY;
		graphicData.m_Pack.m_Config.m_EndX = m_EndX;
		graphicData.m_Pack.m_Config.m_EndY = m_EndY;
	}
};

class RectangleStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;
	uint16_t m_EndX;
	uint16_t m_EndY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType          = 1;
		graphicData.m_Color                = m_Color;
		graphicData.m_Width                = m_Width;
		graphicData.m_StartX               = m_StartX;
		graphicData.m_StartY               = m_StartY;
		graphicData.m_Pack.m_Config.m_EndX = m_EndX;
		graphicData.m_Pack.m_Config.m_EndY = m_EndY;
	}
};

class CircleStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_CenterX;
	uint16_t m_CenterY;
	uint16_t m_Radius;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType            = 2;
		graphicData.m_Color                  = m_Color;
		graphicData.m_Width                  = m_Width;
		graphicData.m_StartX                 = m_CenterX;
		graphicData.m_StartY                 = m_CenterY;
		graphicData.m_Pack.m_Config.m_Radius = m_Radius;
	}
};

class EllipseStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_Width;
	uint16_t m_CenterX;
	uint16_t m_CenterY;
	uint16_t m_HalfAxisX;
	uint16_t m_HalfAxisY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType          = 3;
		graphicData.m_Color                = m_Color;
		graphicData.m_Width                = m_Width;
		graphicData.m_StartX               = m_CenterX;
		graphicData.m_StartY               = m_CenterY;
		graphicData.m_Pack.m_Config.m_EndX = m_HalfAxisX;
		graphicData.m_Pack.m_Config.m_EndY = m_HalfAxisY;
	}
};

class ArcStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_StartAngle;
	uint16_t m_EndAngle;
	uint16_t m_Width;
	uint16_t m_CenterX;
	uint16_t m_CenterY;
	uint16_t m_HalfAxisX;
	uint16_t m_HalfAxisY;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType          = 4;
		graphicData.m_Color                = m_Color;
		graphicData.m_StartAngle           = m_StartAngle;
		graphicData.m_EndAngle             = m_EndAngle;
		graphicData.m_Width                = m_Width;
		graphicData.m_StartX               = m_CenterX;
		graphicData.m_StartY               = m_CenterY;
		graphicData.m_Pack.m_Config.m_EndX = m_HalfAxisX;
		graphicData.m_Pack.m_Config.m_EndY = m_HalfAxisY;
	}
};

class FloatStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_FontSize;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;
	uint16_t m_Precision;
	float m_Value;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType  = 5;
		graphicData.m_Color        = m_Color;
		graphicData.m_StartAngle   = m_FontSize;
		graphicData.m_EndAngle     = m_Precision;
		graphicData.m_Width        = m_Width;
		graphicData.m_StartX       = m_StartX;
		graphicData.m_StartY       = m_StartY;
		graphicData.m_Pack.m_Float = m_Value;
	}
};

class IntegerStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_FontSize;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;
	int32_t m_Value;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType    = 6;
		graphicData.m_Color          = m_Color;
		graphicData.m_StartAngle     = m_FontSize;
		graphicData.m_Width          = m_Width;
		graphicData.m_StartX         = m_StartX;
		graphicData.m_StartY         = m_StartY;
		graphicData.m_Pack.m_Integer = m_Value;
	}
};

class TextStyle : public IClientGraphicElementStyle
{
public:
	uint8_t m_Color;
	uint16_t m_FontSize;
	uint16_t m_Width;
	uint16_t m_StartX;
	uint16_t m_StartY;

	std::string m_Text;

	auto FillGraphicData(GraphicData& graphicData) const -> void override
	{
		graphicData.m_GraphicType = 7;
		graphicData.m_Color       = m_Color;
		graphicData.m_StartAngle  = m_FontSize;
		graphicData.m_Width       = m_Width;
		graphicData.m_StartX      = m_StartX;
		graphicData.m_StartY      = m_StartY;
	}
};

class IClientGraphicElement
{
public:
	virtual ~IClientGraphicElement() = default;
	virtual auto FillGraphicData(GraphicData& graphicData) -> void = 0;
	virtual auto FillText(uint8_t* destTextBuffer, const size_t maxLength) const -> size_t = 0;
	virtual auto IsText() const -> bool = 0;
	virtual auto IsModified() const -> bool = 0;
};

template <typename StyleType>
class ClientGraphicElement : public IClientGraphicElement
{
public:
	explicit ClientGraphicElement(const std::array<uint8_t, 3> name,
	                              const uint8_t layer)
		: m_Layer(layer), m_Style()
	{
		std::copy(name.begin(), name.end(), m_GraphicName);
	}

	virtual ~ClientGraphicElement() = default;

	auto FillGraphicData(GraphicData& graphicData) -> void override
	{
		std::copy(m_GraphicName, m_GraphicName + 3, graphicData.m_GraphicName);
		if (m_IsWaitDelete)
		{
			graphicData.m_OperateType = 3;
			m_IsWaitDelete            = false;
			m_IsInitialized           = false;
		}
		else if (!m_IsInitialized)
		{
			graphicData.m_OperateType = 1;
			//m_IsInitialized           = true;
		}
		else
		{
			graphicData.m_OperateType = 2;
		}
		m_IsInitialized     = !m_IsInitialized;
		graphicData.m_Layer = m_Layer;
		m_Style.FillGraphicData(graphicData);
		m_IsModified = false;
	}

	auto FillText(uint8_t* destTextBuffer, const size_t maxLength) const -> size_t override
	{
		return FillTextHelper(destTextBuffer, maxLength);
	}

	auto GetStyleRef() -> StyleType&
	{
		return m_Style;
	}

	auto Save() -> void
	{
		m_IsModified = true;
	}

	auto Delete() -> void
	{
		m_IsWaitDelete = true;
		m_IsModified   = true;
	}

	auto Layer() const -> uint8_t { return m_Layer; }

	auto IsText() const -> bool override { return std::is_same_v<StyleType, TextStyle>; }

	auto IsModified() const -> bool override { return m_IsModified; }

private:
	uint8_t m_GraphicName[3] = {0, 0, 0};
	uint8_t m_Layer;

	StyleType m_Style;

	bool m_IsInitialized = false;
	bool m_IsModified    = true;
	bool m_IsWaitDelete  = false;

	template <typename ST = StyleType, std::enable_if_t<!std::is_same_v<ST, TextStyle>, int>  = 0>
	auto FillTextHelper(uint8_t* destTextBuffer, const size_t maxLength) const -> size_t
	{
		SPDLOG_WARN("ClientGraphic FillText: FillText be called by shape element");
		return 0;
	}

	template <typename ST = StyleType, std::enable_if_t<std::is_same_v<ST, TextStyle>, int>  = 0>
	auto FillTextHelper(uint8_t* destTextBuffer, const size_t maxLength) const -> size_t
	{
		auto length = m_Style.m_Text.length();
		if (m_Style.m_Text.length() > maxLength)
		{
			SPDLOG_WARN("ClientGraphic FillText: Text length is bigger than maxLength.");
			length = maxLength;
		}
		memcpy(destTextBuffer, m_Style.m_Text.c_str(), length);
		return length;
	}
};

class ClientGraphic
{
public:
	explicit ClientGraphic(const uint16_t id, const IReferee* referee)
		: m_Id(id),
		  m_Referee(referee)
	{
	}

	template <typename StyleType>
	auto AddElement(const uint8_t layer) -> std::shared_ptr<ClientGraphicElement<StyleType>>
	{
		m_Elements.push_back(std::make_shared<ClientGraphicElement<StyleType>>(GetNextGraphicName(), layer));
		return std::dynamic_pointer_cast<ClientGraphicElement<StyleType>>(m_Elements.back());
	}

	auto Render(const bool repaint = false) -> void
	{
	    if (0 == m_Referee->Id())
        {
	        spdlog::warn("ClientGraphic: Referee Id have not been set.");
	        return;
        }

	    bool textProcessed{false};
	    size_t sentBytes = 0;
		for (auto&& element : m_Elements)
		{
			if (element->IsModified() || repaint)
			{
				if (element->IsText())
				{
                    m_ModifiedTextElements.push(element);
				}
				else
				{
                    m_ModifiedTextElements.push(element);
				}
			}
		}
		while (sentBytes < 1280 || // [TODO] 导出宏定义
		       (m_ModifiedGraphElements.empty() && m_ModifiedTextElements.empty()))
        {
            if (!m_ModifiedTextElements.empty() && false == textProcessed)
            {
                auto element = m_ModifiedTextElements.front();
                ClientGraphicTextModify buffer{};
                buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
                buffer.m_Header.m_ReceiverID = m_Id;
                buffer.m_Header.m_SenderID   = m_Referee->Id();
                element->FillGraphicData(buffer.m_GraphicData);
                buffer.m_GraphicData.m_EndAngle = element->FillText(buffer.m_Data, sizeof(buffer.m_Data));
                sentBytes += m_Referee->SendMessage(RefereeBuffer<ClientGraphicTextModify>(buffer));
                m_ModifiedTextElements.pop();
            }
            textProcessed = true;
            if (!m_ModifiedGraphElements.empty() && true == textProcessed)
            {
	            const size_t elementsCount = m_ModifiedGraphElements.size();
                if(elementsCount >= 7) // 7
                {
                    ClientGraphicShapeModify<7> buffer{};
                    buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
					buffer.m_Header.m_ReceiverID = m_Id;
					buffer.m_Header.m_SenderID   = m_Referee->Id();
                    for (size_t i = 0; i < 7; ++i) {
                        auto element = m_ModifiedGraphElements.front();
                        element->FillGraphicData(buffer.m_GraphicData[i]);
                        m_ModifiedGraphElements.pop();
                    }
                    sentBytes += m_Referee->SendMessage(RefereeBuffer<ClientGraphicShapeModify<7>>(buffer));
                }
                else if (elementsCount >= 5) // 5
                {
                    ClientGraphicShapeModify<5> buffer{};
                    buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
                    buffer.m_Header.m_ReceiverID = m_Id;
                    buffer.m_Header.m_SenderID   = m_Referee->Id();
                    for (size_t i = 0; i < 5; ++i) {
                        auto element = m_ModifiedGraphElements.front();
                        element->FillGraphicData(buffer.m_GraphicData[i]);
                        m_ModifiedGraphElements.pop();
                    }
                    sentBytes += m_Referee->SendMessage(RefereeBuffer<ClientGraphicShapeModify<5>>(buffer));
                }
                else if (elementsCount >= 2) // 2
                {
                    ClientGraphicShapeModify<2> buffer{};
                    buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
                    buffer.m_Header.m_ReceiverID = m_Id;
                    buffer.m_Header.m_SenderID   = m_Referee->Id();
                    for (size_t i = 0; i < 7; ++i) {
                        auto element = m_ModifiedGraphElements.front();
                        element->FillGraphicData(buffer.m_GraphicData[i]);
                        m_ModifiedGraphElements.pop();
                    }
                    sentBytes += m_Referee->SendMessage(RefereeBuffer<ClientGraphicShapeModify<2>>(buffer));
                }
                else // 1
                {
                    ClientGraphicShapeModify<1> buffer{};
                    buffer.m_Header.m_DataCmdId  = buffer.DATA_CMD_ID;
                    buffer.m_Header.m_ReceiverID = m_Id;
                    buffer.m_Header.m_SenderID   = m_Referee->Id();
                    for (size_t i = 0; i < 1; ++i) {
                        auto element = m_ModifiedGraphElements.front();
                        element->FillGraphicData(buffer.m_GraphicData[i]);
                        m_ModifiedGraphElements.pop();
                    }
                    sentBytes += m_Referee->SendMessage(RefereeBuffer<ClientGraphicShapeModify<1>>(buffer));
                }
            }
        }
	}

private:
	auto GetNextGraphicName() -> std::array<uint8_t, 3>
	{
		for (size_t i = 0; i < m_Name.size(); ++i)
		{
			if (++m_Name[i] <= '9')
			{
				return m_Name;
			}
			m_Name[i] = '0';
		}
		throw std::bad_alloc(); //[TODO]: Would better define a new exception type.
	}

	uint16_t m_Id = 0;
	std::vector<std::shared_ptr<IClientGraphicElement>> m_Elements{};
	std::array<uint8_t, 3> m_Name{'0', '0', '0'};
	const IReferee* m_Referee;

	std::queue<std::shared_ptr<IClientGraphicElement>> m_ModifiedTextElements{};
	std::queue<std::shared_ptr<IClientGraphicElement>> m_ModifiedGraphElements{};
};

class ClientGraphicManager
{
public:
	OSSIAN_SERVICE_SETUP(ClientGraphicManager(IReferee* referee))
		: m_Referee(referee)
	{
	}

	auto AddOrGetGraphicClient(uint16_t id) -> std::shared_ptr<ClientGraphic>
	{
		const auto it = m_Client.find(id);
		if (it != m_Client.end())
		{
			return it->second;
		}
		m_Client.insert(std::make_pair(id, std::make_shared<ClientGraphic>(id, m_Referee)));
		return m_Client[id];
	}

	auto GetGraphicClients() -> const std::unordered_map<uint16_t, std::shared_ptr<ClientGraphic>>&
    {
	    return m_Client;
    }

private:
	std::unordered_map<uint16_t, std::shared_ptr<ClientGraphic>> m_Client;
	IReferee* m_Referee;
};


#endif // OSSIAN_CLIENT_GRAPHIC
