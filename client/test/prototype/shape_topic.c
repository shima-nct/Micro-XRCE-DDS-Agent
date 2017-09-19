#include "shape_topic.h"

#include <stdio.h>

// -------------------------------------------------------------------------------
//                               TOPIC SERIALIZATION
// -------------------------------------------------------------------------------
void serialize_shape_topic(SerializedBufferHandle* writer, const void* topic_struct)
{
    ShapeTopic* topic = (ShapeTopic*)topic_struct;

    serialize_byte_4(writer, topic->color_length);
    serialize_array(writer, (uint8_t*)topic->color, topic->color_length);
    serialize_byte_4(writer, topic->x);
    serialize_byte_4(writer, topic->y);
    serialize_byte_4(writer, topic->size);
}

void deserialize_shape_topic(SerializedBufferHandle* reader, DynamicBuffer* dynamic_buffer, void* topic_struct)
{
    ShapeTopic* topic = (ShapeTopic*)topic_struct;

    deserialize_byte_4(reader, &topic->color_length);

    topic->color = (char*)request_memory_buffer(dynamic_buffer, topic->color_length);
    deserialize_array(reader, (uint8_t*)topic->color, topic->color_length);
    deserialize_byte_4(reader, &topic->x);
    deserialize_byte_4(reader, &topic->y);
    deserialize_byte_4(reader, &topic->size);
}

uint32_t size_of_shape_topic(const void* topic_struct)
{
    ShapeTopic* topic = (ShapeTopic*)topic_struct;
    return sizeof(topic->color_length)
         + strlen(topic->color) + 1
         + sizeof(topic->x)
         + sizeof(topic->y)
         + sizeof(topic->size);
}

// -------------------------------------------------------------------------------
//                                       UTIL
// -------------------------------------------------------------------------------
void print_shape_topic(ShapeTopic* topic)
{
    printf("\e[1;36m");
    printf("      - color: %s (%u characters)\n", topic->color, topic->color_length);
    printf("      - x: %u\n", topic->x);
    printf("      - y: %u\n", topic->y);
    printf("      - size: %u\n", topic->size);
    printf("\e[0m");
}