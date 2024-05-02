package com.example.presentation.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.presentation.UiState.OrderUiState
import com.example.presentation.databinding.ItemlayoutOrderBinding

class OrderAdapter(private val itemList: List<OrderUiState>) : RecyclerView.Adapter<OrderAdapter.OrderViewHolder>() {
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): OrderViewHolder {
        val inflater = LayoutInflater.from(parent.context)
        val binding = ItemlayoutOrderBinding.inflate(inflater, parent, false)
        return OrderViewHolder(binding)
    }

    override fun onBindViewHolder(holder: OrderViewHolder, position: Int) {
        val item = itemList[position]
        holder.bind(item)
    }

    override fun getItemCount(): Int {
        return itemList.size
    }

    class OrderViewHolder(private val binding: ItemlayoutOrderBinding) : RecyclerView.ViewHolder(binding.root) {

        fun bind(item: OrderUiState) {
            binding.orderItem = item
            binding.executePendingBindings()
        }
    }
}