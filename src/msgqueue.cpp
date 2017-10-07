#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <algorithm>

#include "msgqueue.h"

using namespace std::chrono;

static const MessageQueueOptions DEFAULT_OPTIONS;

static int logError(int code, std::string message) {
    std::cerr << message << " (" << code << ")" << std::endl;
    return code;
}

int MessageQueue::connect()
{
    // Cleanup
    if (client) {
        MQTTClient_destroy(&client);
        client = nullptr;
    }

    if (!backoff_cur) backoff_cur = config.backoff_min;

    // Setup config
    MQTTClient_connectOptions opts = MQTTClient_connectOptions_initializer;
    if (!config.username.empty()) opts.username = config.username.c_str();
    if (!config.password.empty()) opts.password = config.password.c_str();
    opts.keepAliveInterval = config.keep_alive_interval;
    opts.cleansession = true;
    opts.connectTimeout = config.connectTimeout;
    opts.retryInterval = -1;
    
    int error;

    // Create client
    error = MQTTClient_create(
        &client,
        config.uri.c_str(),
        config.client_id.c_str(),
        MQTTCLIENT_PERSISTENCE_NONE,
        nullptr
    );
    if (error) {
        return logError(error, "Unable to create MQTT client");
    }

    // Set callbacks
    error = MQTTClient_setCallbacks(
        client,
        this,
        mqtt_disconnected,
        mqtt_received,
        mqtt_delivered
    );
    if (error) {
        return logError(error, "Unable to set MQTT callbacks");
    }

    std::cout << "Connecting" << std::endl;

    // Connect to server
    error = MQTTClient_connect(client, &opts);
    if (error) {
        logError(error, "Unable to connect to server");
        if (config.reconnect) {
            return reconnect();
        } else {
            return error;
        }
    }
    
    backoff_cur = config.backoff_min;
    std::cout << std::endl << "Connected" << std::endl << std::endl;

    return 0;
}

int MessageQueue::reconnect()
{
    std::cout << "Reconnecting in " << backoff_cur << " ms" << std::endl;
    std::this_thread::sleep_for(milliseconds(backoff_cur));
    backoff_cur += std::rand() % backoff_cur;
    backoff_cur = std::min(backoff_cur, config.backoff_max);
    return connect();
}

int MessageQueue::disconnect()
{
    int error;
    if (client) {
        error = MQTTClient_disconnect(client, config.disconnectTimeout*1000);
        if (error) {
            return logError(error, "Unable to disconnect from server");
        }
    }
    MQTTClient_destroy(&client);
    return 0;
}

int MessageQueue::subscribe(std::string topic, const MessageQueueOptions *options)
{
    if (!options) options = &DEFAULT_OPTIONS;

    int error;
    error = MQTTClient_subscribe(client, topic.c_str(), options->qos);
    if (error) {
        return logError(error, "Unable to subscribe");
    }

    std::cout << "Subscribed to " << topic << std::endl;
    return 0;
}

int MessageQueue::publish(std::string topic, std::string payload, const MessageQueueOptions *options, MessageQueueToken *token)
{
    if (!options) options = &DEFAULT_OPTIONS;

    int error;
    error = MQTTClient_publish(
        client,
        topic.c_str(),
        payload.size(),
        &payload[0],
        options->qos,
        options->retained,
        token
    );
    if (error) {
        return logError(error, "Unable to publish");
    }
    return 0;
}


void MessageQueue::handleReceived(std::string topic, std::string payload)
{
    std::cout << "> " << topic << ": " << payload << std::endl;
    if (onReceived) onReceived(topic, payload);
}

void MessageQueue::handleDelivered(MessageQueueToken token)
{
    std::cout << "Delivered: " << token << std::endl;
    if (onDelivered) onDelivered(token);
}

void MessageQueue::handleDisconnected(std::string cause)
{
    std::string postfix = cause.empty() ? "" : ": " + cause;
    std::cout << "Connection lost" << postfix << std::endl;
    if (onDisconnected) onDisconnected(cause);
    reconnect();
}


int mqtt_received(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    auto queue = static_cast<MessageQueue*>(context);

    std::string topic, payload;

    if (topicLen) {
        topic.resize(topicLen);
        memcpy(&topic[0], topicName, topicLen);
    } else {
        topic = topicName;
    }

    if (message->payloadlen) {
        payload.resize(message->payloadlen);
        memcpy(&payload[0], message->payload, message->payloadlen);
    } else {
        payload = "";
    }

    queue->handleReceived(topic, payload);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void mqtt_delivered(void *context, MQTTClient_deliveryToken dt)
{
    auto queue = static_cast<MessageQueue*>(context);
    queue->handleDelivered(dt);
}

void mqtt_disconnected(void *context, char *cause)
{
    auto queue = static_cast<MessageQueue*>(context);
    std::string message = "";
    if (cause) message = cause;
    queue->handleDisconnected(message);
}

