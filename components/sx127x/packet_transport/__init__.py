import esphome.codegen as cg
from esphome.components.packet_transport import (
    PacketTransport,
    new_packet_transport,
    transport_schema,
)
from esphome.components.sx127x import CONF_SX127X_ID, SX127x, SX127xListener, sx127x_ns
import esphome.config_validation as cv
from esphome.cpp_types import PollingComponent

SX127xTransport = sx127x_ns.class_(
    "SX127xTransport", PacketTransport, PollingComponent, SX127xListener
)

CONFIG_SCHEMA = transport_schema(SX127xTransport).extend(
    {
        cv.GenerateID(CONF_SX127X_ID): cv.use_id(SX127x),
    }
)


async def to_code(config):
    var, providers = await new_packet_transport(config)
    sx127x = await cg.get_variable(config[CONF_SX127X_ID])
    cg.add(var.set_parent(sx127x))
    cg.add(sx127x.register_listener(var))
