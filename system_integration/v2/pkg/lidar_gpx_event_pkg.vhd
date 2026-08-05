library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;

package lidar_gpx_event_pkg is

    constant C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH : positive := 63;
    constant C_GPX_SHOT_START_PAYLOAD_WIDTH   : positive :=
        C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH + 32;
    constant C_GPX_SHOT_CONTEXT_WIDTH         : positive :=
        C_GPX_SHOT_START_PAYLOAD_WIDTH + 1;

    subtype gpx_shot_request_payload_t is std_logic_vector(
        C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH - 1 downto 0);
    subtype gpx_shot_start_payload_t is std_logic_vector(
        C_GPX_SHOT_START_PAYLOAD_WIDTH - 1 downto 0);
    subtype gpx_shot_context_t is std_logic_vector(
        C_GPX_SHOT_CONTEXT_WIDTH - 1 downto 0);

    type gpx_raw_event_kind_t is (
        GPX_RAW_DATA,
        GPX_RAW_IFIFO1_DONE,
        GPX_RAW_DRAIN_DONE,
        GPX_RAW_TIMEOUT
    );

    subtype gpx_chip_index_t is unsigned(1 downto 0);

    type gpx_raw_event_t is record
        valid         : std_logic;
        kind          : gpx_raw_event_kind_t;
        chip_index    : gpx_chip_index_t;
        ififo_id      : std_logic;
        raw_word      : gpx_bus_data_t;
        faulted       : std_logic;
        timeout_cause : std_logic_vector(2 downto 0);
        shot_context  : shot_start_event_t;
        chip_shot_seq : unsigned(15 downto 0);
    end record gpx_raw_event_t;

    constant C_GPX_RAW_EVENT_PAYLOAD_WIDTH : positive :=
        2 + 2 + 1 + C_GPX_BUS_DATA_WIDTH + 1 + 3 +
        C_GPX_SHOT_CONTEXT_WIDTH + 16;

    subtype gpx_raw_event_payload_t is std_logic_vector(
        C_GPX_RAW_EVENT_PAYLOAD_WIDTH - 1 downto 0);

    constant C_GPX_RAW_EVENT_IDLE : gpx_raw_event_t := (
        valid         => '0',
        kind          => GPX_RAW_DATA,
        chip_index    => (others => '0'),
        ififo_id      => '0',
        raw_word      => (others => '0'),
        faulted       => '0',
        timeout_cause => (others => '0'),
        shot_context  => C_SHOT_START_EVENT_IDLE,
        chip_shot_seq => (others => '0')
    );

    function fn_pack_shot_request(
        value : shot_request_t
    ) return gpx_shot_request_payload_t;

    function fn_unpack_shot_request(
        value : gpx_shot_request_payload_t
    ) return shot_request_t;

    function fn_pack_shot_start(
        value : shot_start_event_t
    ) return gpx_shot_start_payload_t;

    function fn_unpack_shot_start(
        value : gpx_shot_start_payload_t
    ) return shot_start_event_t;

    function fn_pack_shot_context(
        value : shot_start_event_t
    ) return gpx_shot_context_t;

    function fn_unpack_shot_context(
        value : gpx_shot_context_t
    ) return shot_start_event_t;

    function fn_pack_raw_event(
        value : gpx_raw_event_t
    ) return gpx_raw_event_payload_t;

    function fn_unpack_raw_event(
        value : gpx_raw_event_payload_t
    ) return gpx_raw_event_t;

end package lidar_gpx_event_pkg;

package body lidar_gpx_event_pkg is

    function fn_pack_shot_request(
        value : shot_request_t
    ) return gpx_shot_request_payload_t is
        variable result : gpx_shot_request_payload_t := (others => '0');
        variable index  : natural := 0;
    begin
        result(index) := value.valid;
        index := index + 1;
        result(index + 2 downto index) := std_logic_vector(value.face_index);
        index := index + 3;
        result(index + C_POSITION_WIDTH - 1 downto index) :=
            std_logic_vector(value.position);
        index := index + C_POSITION_WIDTH;
        if value.direction = DIRECTION_CCW then
            result(index) := '1';
        end if;
        index := index + 1;
        result(index + value.shot_index'length - 1 downto index) :=
            std_logic_vector(value.shot_index);
        index := index + value.shot_index'length;
        result(index) := value.last_in_face;
        index := index + 1;
        result(index) := value.source_sim;
        index := index + 1;
        result(index + value.source_latency_clks'length - 1 downto index) :=
            std_logic_vector(value.source_latency_clks);
        index := index + value.source_latency_clks'length;
        result(index) := value.source_latency_valid;
        index := index + 1;
        result(index + value.active_version'length - 1 downto index) :=
            std_logic_vector(value.active_version);
        return result;
    end function fn_pack_shot_request;

    function fn_unpack_shot_request(
        value : gpx_shot_request_payload_t
    ) return shot_request_t is
        variable result : shot_request_t := C_SHOT_REQUEST_IDLE;
        variable index  : natural := 0;
    begin
        result.valid := value(index);
        index := index + 1;
        result.face_index := unsigned(value(index + 2 downto index));
        index := index + 3;
        result.position := unsigned(
            value(index + C_POSITION_WIDTH - 1 downto index));
        index := index + C_POSITION_WIDTH;
        if value(index) = '1' then
            result.direction := DIRECTION_CCW;
        else
            result.direction := DIRECTION_CW;
        end if;
        index := index + 1;
        result.shot_index := unsigned(
            value(index + result.shot_index'length - 1 downto index));
        index := index + result.shot_index'length;
        result.last_in_face := value(index);
        index := index + 1;
        result.source_sim := value(index);
        index := index + 1;
        result.source_latency_clks := unsigned(
            value(index + result.source_latency_clks'length - 1 downto index));
        index := index + result.source_latency_clks'length;
        result.source_latency_valid := value(index);
        index := index + 1;
        result.active_version := unsigned(
            value(index + result.active_version'length - 1 downto index));
        return result;
    end function fn_unpack_shot_request;

    function fn_pack_shot_start(
        value : shot_start_event_t
    ) return gpx_shot_start_payload_t is
        variable result : gpx_shot_start_payload_t := (others => '0');
    begin
        result(C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH - 1 downto 0) :=
            fn_pack_shot_request(value.request);
        result(result'high downto C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH) :=
            std_logic_vector(value.fire_to_t0_clks);
        return result;
    end function fn_pack_shot_start;

    function fn_unpack_shot_start(
        value : gpx_shot_start_payload_t
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.request := fn_unpack_shot_request(
            value(C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH - 1 downto 0));
        result.fire_to_t0_clks := unsigned(
            value(value'high downto C_GPX_SHOT_REQUEST_PAYLOAD_WIDTH));
        return result;
    end function fn_unpack_shot_start;

    function fn_pack_shot_context(
        value : shot_start_event_t
    ) return gpx_shot_context_t is
        variable result : gpx_shot_context_t := (others => '0');
    begin
        result(C_GPX_SHOT_START_PAYLOAD_WIDTH - 1 downto 0) :=
            fn_pack_shot_start(value);
        result(result'high) := value.valid;
        return result;
    end function fn_pack_shot_context;

    function fn_unpack_shot_context(
        value : gpx_shot_context_t
    ) return shot_start_event_t is
        variable result : shot_start_event_t;
    begin
        result := fn_unpack_shot_start(
            value(C_GPX_SHOT_START_PAYLOAD_WIDTH - 1 downto 0));
        result.valid := value(value'high);
        return result;
    end function fn_unpack_shot_context;

    function fn_pack_raw_event(
        value : gpx_raw_event_t
    ) return gpx_raw_event_payload_t is
        variable result : gpx_raw_event_payload_t := (others => '0');
        variable index  : natural := 0;
    begin
        result(index + 1 downto index) := std_logic_vector(
            to_unsigned(gpx_raw_event_kind_t'pos(value.kind), 2));
        index := index + 2;
        result(index + 1 downto index) := std_logic_vector(value.chip_index);
        index := index + 2;
        result(index) := value.ififo_id;
        index := index + 1;
        result(index + C_GPX_BUS_DATA_WIDTH - 1 downto index) := value.raw_word;
        index := index + C_GPX_BUS_DATA_WIDTH;
        result(index) := value.faulted;
        index := index + 1;
        result(index + 2 downto index) := value.timeout_cause;
        index := index + 3;
        result(index + C_GPX_SHOT_CONTEXT_WIDTH - 1 downto index) :=
            fn_pack_shot_context(value.shot_context);
        index := index + C_GPX_SHOT_CONTEXT_WIDTH;
        result(index + value.chip_shot_seq'length - 1 downto index) :=
            std_logic_vector(value.chip_shot_seq);
        return result;
    end function fn_pack_raw_event;

    function fn_unpack_raw_event(
        value : gpx_raw_event_payload_t
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
        variable index  : natural := 0;
        variable kind_value : natural;
    begin
        kind_value := to_integer(unsigned(value(index + 1 downto index)));
        result.kind := gpx_raw_event_kind_t'val(kind_value);
        index := index + 2;
        result.chip_index := unsigned(value(index + 1 downto index));
        index := index + 2;
        result.ififo_id := value(index);
        index := index + 1;
        result.raw_word := value(
            index + C_GPX_BUS_DATA_WIDTH - 1 downto index);
        index := index + C_GPX_BUS_DATA_WIDTH;
        result.faulted := value(index);
        index := index + 1;
        result.timeout_cause := value(index + 2 downto index);
        index := index + 3;
        result.shot_context := fn_unpack_shot_context(
            value(index + C_GPX_SHOT_CONTEXT_WIDTH - 1 downto index));
        index := index + C_GPX_SHOT_CONTEXT_WIDTH;
        result.chip_shot_seq := unsigned(
            value(index + result.chip_shot_seq'length - 1 downto index));
        return result;
    end function fn_unpack_raw_event;

end package body lidar_gpx_event_pkg;
