---
title: 理解Prometheus
key: 20190716
tags:
  - Monitoring
  - 监控
  - Prometheus
---

## 概述

[Prometheus](https://prometheus.io/) 是一款监控软件。真正学习和使用过Promethues后，发现它对于监控业务的理解极其透彻。理解Promethues更像是在了解监控的本质。

## 数据模型

我们要监控的对象是什么？

理解Prometheus的第一步是理解它的数据模型。

https://prometheus.io/blog/2016/07/23/pull-does-not-scale-or-does-it/