#ifndef OSSIAN_REFEREE_HPP
#define OSSIAN_REFEREE_HPP

#include <ossian/Factory.hpp>
#include <ossian/io/UART.hpp>
#include <ossian/MultiThread.hpp>
#include <ossian/IOData.hpp>
#include <spdlog/spdlog.h>

#include <mutex>
#include <typeindex>

#pragma pack(push,1)
struct FrameHeaderWithCmd
{
	uint8_t m_SOF;         ///< 起始字节(0xA5)
	uint16_t m_DataLength; ///< 数据长度
	uint8_t m_Seq;         ///< 包序号
	uint8_t m_CRC8;        ///< 帧头CRC8校验
	uint16_t m_CmdId;      ///< 命令码
};

struct FrameTail
{
	uint16_t m_CRC16; ///< 整包校验
};

/**
 * @brief 比赛状态数据，1Hz 周期发送(0x0001)
 */
struct MatchStatus
{
	static constexpr uint16_t cmdId = 0x0001;
	static constexpr size_t length  = 3;

	uint8_t m_GameType : 4;     ///< 比赛类型：1:机甲大师赛 2:单项赛 3:人工智能挑战赛
	uint8_t m_GameProgress : 4; ///< 当前比赛阶段：0:未开始比赛 1:准备阶段 2:自检阶段 3:5s倒计时 4:对战中 5:比赛结算中
	uint16_t m_StageRemainTime; ///< 当前阶段剩余时间(s)
};

/**
 * @brief 比赛结果数据，比赛结束后发送(0x0002)
 */
struct MatchResult
{
	static constexpr uint16_t cmdId = 0x0002;
	static constexpr size_t length  = 1;

	uint8_t m_Winner; ///< 比赛结果：0:平局 1:红方胜利 2:蓝方胜利
};

/**
 * @brief 比赛机器人血量数据，1Hz 周期发送(0x0003)
 */
struct RobotHP
{
	static constexpr uint16_t cmdId = 0x0003;
	static constexpr size_t length  = 28;

	uint16_t m_Red1HP;     ///< 红1英雄
	uint16_t m_Red2HP;     ///< 红2工程
	uint16_t m_Red3HP;     ///< 红3步兵
	uint16_t m_Red4HP;     ///< 红4步兵
	uint16_t m_Red5HP;     ///< 红5步兵
	uint16_t m_Red7HP;     ///< 红7哨兵
	uint16_t m_RedBaseHP;  ///< 红基地
	uint16_t m_Blue1HP;    ///< 蓝1英雄
	uint16_t m_Blue2HP;    ///< 蓝2工程
	uint16_t m_Blue3HP;    ///< 蓝3步兵
	uint16_t m_Blue4HP;    ///< 蓝4步兵
	uint16_t m_Blue5HP;    ///< 蓝5步兵
	uint16_t m_Blue7HP;    ///< 蓝7哨兵
	uint16_t m_BlueBaseHP; ///< 蓝基地
};

/**
 * @brief 场地事件数据，事件改变后发送(0x0101)
 */
struct VenueEvent
{
	static constexpr uint16_t cmdId = 0x0101;
	static constexpr size_t length  = 4;

	uint8_t m_TarmacOccupation : 2; ///< 己方停机坪占领状态：0 为无机器人占领；1 为空中机器人已占领但未停桨；2 为空中机器人已占领并停桨

	bool m_HealingOccupation1 : 1; ///< 己方补给站 1 号补血点占领状态，1 为已占领
	bool m_HealingOccupation2 : 1; ///< 己方补给站 2 号补血点占领状态，1 为已占领
	bool m_HealingOccupation3 : 1; ///< 己方补给站 3 号补血点占领状态，1 为已占领

	bool m_StrikePointOccupation : 1; ///< 为打击点占领状态，1 为占领
	bool m_SmallEnergyActivation : 1; ///< 为小能量机关激活状态，1 为已激活
	bool m_BigEnergyActivation : 1;   ///< 为大能量机关激活状态，1 为已激活

	bool m_PassOccupation : 1;           ///< 己方关口占领状态，1 为已占领
	bool m_BunkerOccupation : 1;         ///< 己方碉堡占领状态，1 为已占领
	bool m_ResourceIslandOccupation : 1; ///< 己方资源岛占领状态，1 为已占领

	bool m_BaseShieldStatus : 1;   ///< 己方基地护盾状态：1 为基地有虚拟护盾血量；0 为基地无虚拟护盾血量
	uint8_t m_Reserve0 : 4;        ///< 保留
	uint8_t m_Reserve1 : 8;        ///< 保留
	uint8_t m_Reserve2 : 4;        ///< 保留
	uint8_t m_RedDefenseBuff : 2;  ///< ICRA 红方防御加成：0：防御加成未激活；1：防御加成 5s 触发激活中；2：防御加成已激活
	uint8_t m_BlueDefenseBuff : 2; ///< ICRA 蓝方防御加成：0：防御加成未激活；1：防御加成 5s 触发激活中；2：防御加成已激活
};

/**
 * @brief 场地补给站动作标识数据，动作改变后发送(0x0102)
 */
struct SupplyStation
{
	static constexpr uint16_t cmdId = 0x0102;
	static constexpr size_t length  = 3;

	uint8_t m_SupplyProjectileId; ///< 补给站口ID：1:1号补给口 2:2号补给口

	/**
	 * @brief 补弹机器人 ID
	 * 0 为当前无机器人补弹
	 * 1 为红方英雄机器人补弹
	 * 2 为红方工程机器人补弹
	 * 3/4/5 为红方步兵机器人补弹
	 * 11 为蓝方英雄机器人补弹
	 * 12 为蓝方工程机器人补弹
	 * 13/14/15 为蓝方步兵机器人补弹
	 */
	uint8_t m_SupplyRobotId;

	uint8_t m_SupplyProjectileStep; ///< 出弹口开闭状态：0 为关闭，1 为弹丸准备中，2 为弹丸下落

	uint8_t m_SupplyProjectileNum; ///< 补弹数量： 50：50 颗弹丸；100：100 颗弹丸；150：150 颗弹丸；200：200 颗弹丸。 
};

/**
 * @brief 请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）(0x103)
 */
struct SupplyStationQuery
{
	static constexpr uint16_t cmdId = 0x0103;
	static constexpr size_t length  = 2;

	uint8_t m_SupplyProjectileId; ///< 补给站补弹口ID：1:1号补给口

	/**
	 * @brief 补弹机器人 ID
	 * 1 为红方英雄机器人补弹
	 * 2 为红方工程机器人补弹，
	 * 3/4/5 为红方步兵机器人补弹
	 * 11 为蓝方英雄机器人补弹
	 * 12 为蓝方工程机器人补弹
	 * 13/14/15 为蓝方步兵机器人补弹
	 */
	uint8_t m_SupplyRobotId;

	uint8_t m_SupplyNum; ///< 补弹数目：50:请求50颗弹丸下落
};

/**
 * @brief 裁判警告数据，警告发生后发送(0x0104)
 */
struct RefereeWarning
{
	static constexpr uint16_t cmdId = 0x0104;
	static constexpr size_t length  = 2;

	uint8_t m_Level;       ///< 警告等级
	uint8_t m_FoulRobotId; ///< 犯规机器人ID：1级以及5级警告时，机器人ID为0，二三四级警告时，机器人ID为犯规机器人ID
};

/**
 * @brief 机器人状态数据，10Hz 周期发送(0x0201)
 */
struct RobotStatus
{
	static constexpr uint16_t cmdId = 0x0201;
	static constexpr size_t length  = 15;

	/**
	 * @brief 机器人 ID：
	 * 1：红方英雄机器人；
	 * 2：红方工程机器人；
	 * 3/4/5：红方步兵机器人；
	 * 6：红方空中机器人；
	 * 7：红方哨兵机器人；
	 * 11：蓝方英雄机器人；
	 * 12：蓝方工程机器人；
	 * 13/14/15：蓝方步兵机器人；
	 * 16：蓝方空中机器人；
	 * 17：蓝方哨兵机器人
	 */
	uint8_t m_RobotId;
	uint8_t m_RobotLevel;                  ///< 机器人等级 1：一级；2：二级；3：三级
	uint16_t m_RemainHp;                   ///< 机器人剩余血量
	uint16_t m_MaxHp;                      ///< 机器人上限血量
	uint16_t m_Shooter17CoolingRate;       ///< 机器人 17mm 枪口每秒冷却值 
	uint16_t m_Shooter17HeatLimit;         ///< 机器人 17mm 枪口热量上限 
	uint16_t m_Shooter42CoolingRate;       ///< 机器人 42mm 枪口每秒冷却值 
	uint16_t m_Shooter42HeatLimit;         ///< 机器人 42mm 枪口热量上限 
	uint8_t m_MainsPowerGimbalOutput : 1;  ///< 主控电源输出情况：0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
	uint8_t m_MainsPowerChassisOutput : 1; ///< 主控电源输出情况：1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出； 
	uint8_t m_MainsPowerShooterOutput : 1; ///< 主控电源输出情况：2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
};


/**
 * @brief 实时功率热量数据，50Hz 周期发送(0x0202)
 */
struct PowerHeatData
{
	static constexpr uint16_t cmdId = 0x0202;
	static constexpr size_t length  = 14;

	uint16_t m_ChassisVolt;        ///< 底盘输出电压 单位 毫伏 
	uint16_t m_ChassisCurrent;     ///< 底盘输出电流 单位 毫安 
	float m_ChassisPower;          ///< （四字节）底盘输出功率 单位 W 瓦
	uint16_t m_ChassisPowerBuffer; ///< 底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J 
	uint16_t m_Shooter17Heat;      ///< 17mm 枪口热量 
	uint16_t m_Shooter42Heat;      ///< 42mm 枪口热量 
};

/**
 * @brief 机器人位置数据，10Hz 发送(0x0203)
 */
struct RobotPosition
{
	static constexpr uint16_t cmdId = 0x0203;
	static constexpr size_t length  = 16;

	float m_X;   ///< （四字节）位置 x 坐标，单位 m 
	float m_Y;   ///< （四字节）位置 y 坐标，单位 m 
	float m_Z;   ///< （四字节）位置 z 坐标，单位 m 
	float m_Yaw; ///< （四字节）位置枪口，单位度
};

/**
 * @brief 机器人增益数据，增益状态改变后发送(0x0204)
 */
struct RobotRuneBuff
{
	static constexpr uint16_t cmdId = 0x0204;
	static constexpr size_t length  = 1;

	bool m_HealingStatus : 1;             ///< 机器人血量补血状态
	bool m_BarrelCoolingAcceleration : 1; ///< 枪口热量冷却加速
	bool m_DefenseBuff : 1;               ///< 机器人防御加成
	bool m_AttackBuff : 1;                ///< 机器人攻击加成
	uint8_t m_Reserve : 4;                ///< 保留
};

/**
 * @brief 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送(0x0205)
 */
struct AerialRobotStatus
{
	static constexpr uint16_t cmdId = 0x0205;
	static constexpr size_t length  = 3;

	uint8_t m_EnergyPoint; ///< 积累的能量点
	uint8_t m_AttackTime;  ///< 可攻击时间 单位 s。30s 递减至 0
};

/**
 * @brief 伤害状态数据，伤害发生后发送(0x0206)
 */
struct DamageStatus
{
	static constexpr uint16_t cmdId = 0x0206;
	static constexpr size_t length  = 1;

	/**
	 * @brief 装甲ID
	 * 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为0-4号代表机器人的五个装甲片
	 * 其他血量变化类型，该变量数值为 0
	 */
	uint8_t m_ArmorId : 4;

	/**
	 * @brief 血量变化类型
	 * 0x0 装甲伤害扣血；
	 * 0x1 模块掉线扣血；
	 * 0x2 超射速扣血；
	 * 0x3 超枪口热量扣血；
	 * 0x4 超底盘功率扣血；
	 * 0x5 装甲撞击扣血
	 */
	uint8_t m_HurtType : 4;
};

/**
 * @brief 实时射击数据，弹丸发射后发送(0x0207)
 */
struct ShootData
{
	static constexpr uint16_t cmdId = 0x0207;
	static constexpr size_t length  = 6;

	uint8_t m_BulletType; ///< 弹丸类型: 1：17mm 弹丸 2：42mm 弹丸 
	uint8_t m_BulletFreq; ///< 弹丸射频 单位 Hz
	float m_BulletSpeed;  ///< （四字节）弹丸射速 单位 m/s 
};

/**
 * @brief 弹丸剩余发射数，仅空中机器人以及哨兵机器人主控发送该数据，1Hz 周期发送(0x0208)
 */
struct BulletRemain
{
	static constexpr uint16_t cmdId = 0x0208;
	static constexpr size_t length  = 2;

	uint16_t m_BulletRemainingNum; ///< 弹丸剩余发射数目
};

template <typename MessageType>
struct RefereeMessage
{
	static constexpr size_t length = sizeof(FrameHeaderWithCmd) + MessageType::length + sizeof(FrameTail);

	FrameHeaderWithCmd m_Header;
	MessageType m_Payload;
	FrameTail m_Tail;
};

#pragma pack(pop)

template <typename T>
struct IsValidModel
{
	static constexpr bool value = {sizeof(T) == T::length};
};

template <int N, typename... Ts>
using NThTypeOf = typename std::tuple_element<N, std::tuple<Ts...>>::type;

class IReferee
{
public:
	virtual auto AddReferee(std::string location) -> void = 0;
};

template <typename Mutex = std::mutex, typename ...MessageTypes>
class Referee : public IReferee, public ossian::IODataBuilder<Mutex, MessageTypes...>
{
public:
	OSSIAN_SERVICE_SETUP(Referee(ossian::UARTManager* uartManager,
		ossian::IOData<MessageTypes>*...listeners))
		: m_UARTManager(uartManager)
		  , m_Container(std::make_tuple(listeners...))
	{
	}

	virtual ~Referee() = default;

	auto AddReferee(std::string location) -> void override
	{
		using namespace ossian::UARTProperties;
		m_UARTManager->AddDevice(location,
		                         Baudrate::R115200,
		                         FlowControl::FlowControlNone,
		                         DataBits::DataBits8,
		                         StopBits::StopBits1,
		                         Parity::ParityNone)
		             ->SetCallback(
			             [this](const std::shared_ptr<ossian::BaseDevice>& device,
			                    const size_t length,
			                    const uint8_t* data)
			             {
				             spdlog::trace("Referee Receive: {}", length);
				             ParseReferee(data, length);
			             });
	}

private:
	template <typename T>
	struct IsValidModel
	{
		static constexpr bool value = {sizeof(T) == T::length};
	};

	template <int N, typename... Ts>
	using NThTypeOf = typename std::tuple_element<N, std::tuple<Ts...>>::type;

	template <typename T, typename Tuple>
	struct IndexOf
	{
		static_assert(!std::is_same<Tuple, std::tuple<>>::value, "Could not find T in given Tuple");
	};

	template <typename T, typename... Types>
	struct IndexOf<T, std::tuple<T, Types...>>
	{
		static constexpr std::size_t value = 0;
	};

	template <typename T, typename U, typename... Types>
	struct IndexOf<T, std::tuple<U, Types...>>
	{
		static constexpr std::size_t value = 1 + IndexOf<T, std::tuple<Types...>>::value;
	};

	template <typename T, typename Tuple>
	struct CountOf
	{
	};

	template <typename T>
	struct CountOf<T, std::tuple<>>
	{
		static constexpr std::size_t value = 0;
	};

	template <typename T, typename ...Types>
	struct CountOf<T, std::tuple<T, Types...>>
	{
		static constexpr std::size_t value = 1 + CountOf<T, std::tuple<Types...>>::value;
	};

	template <typename T, typename U, typename ...Types>
	struct CountOf<T, std::tuple<U, Types...>>
	{
		static constexpr std::size_t value = 0 + CountOf<T, std::tuple<Types...>>::value;
	};

	using Container = std::tuple<ossian::IOData<MessageTypes>*...>;
	static_assert((IsValidModel<MessageTypes>::value || ...), "There is a invalid model");
	static_assert(((CountOf<MessageTypes, std::tuple<MessageTypes>>::value == 1) && ...),
		"Redefined message in MessageTypes");

	template <typename MessageType, size_t Index>
	auto ReadData(const uint8_t* data, const size_t length) -> size_t
	{
		const auto cmdId{reinterpret_cast<const FrameHeaderWithCmd*>(data)->m_CmdId};
		if (MessageType::cmdId != cmdId)
		{
			return 0;
		}
		if (length < RefereeMessage<MessageType>::length)
		{
			throw std::runtime_error("Buffer is incomplete");
		}
		{
			//[TODO]: Check the thread safe rule for the separate lock design
			//std::lock_guard<Mutex> guard{std::get<Index>(m_Mutexes)};
			std::get<Index>(m_Container)->Set(reinterpret_cast<const RefereeMessage<MessageType>*>(data)->m_Payload);
		}
		spdlog::trace("Matched message: {:x}\t Message Length: {}\t Matched: {}",
		              MessageType::cmdId, RefereeMessage<MessageType>::length, Index);
		return RefereeMessage<MessageType>::length;
	}

	template <size_t ...Index>
	auto ReadPack(const uint8_t* data, const size_t length, std::index_sequence<Index...>) -> size_t
	{
		try
		{
			auto readLength{(ReadData<NThTypeOf<Index, MessageTypes...>, Index>(data, length) + ...)};
			if (0 == readLength && length > sizeof(FrameHeaderWithCmd))
			{
				readLength = sizeof(FrameHeaderWithCmd)
				             + sizeof(FrameTail)
				             + reinterpret_cast<const FrameHeaderWithCmd*>(data)->m_DataLength;
			}
			return readLength > length ? 0 : readLength;
		}
		catch (std::runtime_error& err)
		{
			spdlog::warn("{}", err.what());
			return 0;
		}
	}

	auto ParseReferee(const uint8_t* data, const size_t length)
	{
		size_t remainLength = length;
		while (remainLength > 0)
		{
			auto currentPos{data + (length - remainLength)};
			if (currentPos[0] != 0xA5)
			{
				--remainLength;
				continue;
			}
			//[TODO]: Make sure remainLength won't be a minus value
			auto readLength = ReadPack(currentPos,
			                           remainLength,
			                           std::index_sequence_for<MessageTypes...>());
			remainLength -= readLength > 0 ? readLength : remainLength; ///< 避免因无法匹配进入死循环
		}
	}

	ossian::UARTManager* m_UARTManager;
	Container m_Container;
};

template <typename ...MessageTypes>
using RefereeMt = Referee<std::mutex, MessageTypes...>;

template <typename ...MessageTypes>
using RefereeSt = Referee<ossian::null_mutex, MessageTypes...>;

using RefereeAllMessagesMt = RefereeMt<BulletRemain,
                                       ShootData,
                                       DamageStatus,
                                       AerialRobotStatus,
                                       RobotRuneBuff,
                                       RobotPosition,
                                       PowerHeatData,
                                       RobotStatus,
                                       RefereeWarning,
                                       SupplyStation,
                                       VenueEvent,
                                       RobotHP,
                                       MatchStatus,
                                       MatchResult>;

using RefereeAllMessagesSt = RefereeSt<BulletRemain,
                                       ShootData,
                                       DamageStatus,
                                       AerialRobotStatus,
                                       RobotRuneBuff,
                                       RobotPosition,
                                       PowerHeatData,
                                       RobotStatus,
                                       RefereeWarning,
                                       SupplyStation,
                                       VenueEvent,
                                       RobotHP,
                                       MatchStatus,
                                       MatchResult>;

#endif // OSSIAN_REFEREE_HPP
