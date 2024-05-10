package com.example.domain.usecase

import com.example.domain.model.TurtleClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class TurtleUseCase @Inject constructor(
    private val repository: Repository,
) {

    suspend operator fun invoke(): Result<List<TurtleClass>> =
        repository.turtleRepo()
}