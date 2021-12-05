#pragma once

#include <functional>

#include "MQTTClient.h"

typedef int MessageQueueToken;

struct MessageQueueConfig {
    std::string uri;
    std::string client_id;
    std::string username;
    std::string password;

    bool reconnect = true;

    int keep_alive_interval = 15;

    int backoff_min = 50;
    int backoff_max = 2000;

    int connectTimeout = 1;
    int disconnectTimeout = 5;
};

struct MessageQueueOptions {
    int qos = 0;
    bool retained = false;
};

class MessageQueue {
protected:
    MQTTClient client = nullptr;
    std::vector<std::pair<std::string, MessageQueueOptions>> subscriptions;

    int backoff_cur = 0;

    void handleReceived(std::string topic, std::string payload);
    void handleDelivered(MessageQueueToken token);
    void handleDisconnected(std::string cause);
    friend int mqtt_received(void *context, char *topicName, int topicLen, MQTTClient_message *message);
    friend void mqtt_delivered(void *context, MQTTClient_deliveryToken dt);
    friend void mqtt_disconnected(void *context, char *cause);

    int reconnect();

public:
    MessageQueueConfig config;
    
    std::function<void(std::string topic, std::string payload)> onReceived;
    std::function<void(MessageQueueToken token)> onDelivered;
    std::function<void(std::string cause)> onDisconnected;

    int connect();
    int disconnect();

    int subscribe(std::string topic, const MessageQueueOptions *options = nullptr);
    int publish(std::string topic, std::string payload, const MessageQueueOptions *options = nullptr, MessageQueueToken *token = nullptr);
};
