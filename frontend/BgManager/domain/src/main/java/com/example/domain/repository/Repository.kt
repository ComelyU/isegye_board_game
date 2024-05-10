package com.example.domain.repository

import com.example.domain.model.DeliverClass
import com.example.domain.model.DeliverResponseClass
import com.example.domain.model.GameClass
import com.example.domain.model.OrderClass
import com.example.domain.model.TurtleClass

interface Repository {
    suspend fun turtleRepo(): Result<List<TurtleClass>>

    suspend fun orderRepo(): Result<List<OrderClass>>

    suspend fun gameRepo(): Result<List<GameClass>>

    suspend fun deliverRepo(deliverClass: DeliverClass): Result<DeliverResponseClass>
}