package com.example.domain.usecase

import com.example.domain.model.TurtleBotClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class TurtleBotUseCase @Inject constructor(
    private val repository: Repository,
) {

    suspend operator fun invoke(): Result<TurtleBotClass> =
        repository.turtleRepo()
}
