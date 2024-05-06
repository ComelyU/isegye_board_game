package com.example.presentation.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.presentation.databinding.ItemlayoutOrderDetailBinding
import com.example.presentation.ui.OrderDetailState

class OrderDetailAdapter (private val orderDetails: List<OrderDetailState>) :
    RecyclerView.Adapter<OrderDetailAdapter.OrderDetailViewHolder>() {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): OrderDetailViewHolder {
        val inflater = LayoutInflater.from(parent.context)
        val binding = ItemlayoutOrderDetailBinding.inflate(inflater, parent, false)
        return OrderDetailViewHolder(binding)
    }

    override fun onBindViewHolder(holder: OrderDetailViewHolder, position: Int) {
        holder.bind(orderDetails[position])
    }

    override fun getItemCount(): Int {
        return orderDetails.size
    }

    inner class OrderDetailViewHolder(private val binding: ItemlayoutOrderDetailBinding) :
        RecyclerView.ViewHolder(binding.root) {

        fun bind(orderDetail: OrderDetailState) {
            binding.orderDetailItem = orderDetail
            binding.executePendingBindings()
        }
    }
}