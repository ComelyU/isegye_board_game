package com.example.domain.usecase

import com.example.domain.model.DeliverClass
import com.example.domain.model.DeliverResponseClass
import com.example.domain.repository.Repository
import javax.inject.Inject

class DeliverUseCase @Inject constructor(
    private val repository: Repository,
) {

    suspend operator fun invoke(deliverClass: DeliverClass): Result<DeliverResponseClass> =
        repository.deliverRepo(deliverClass)
}
