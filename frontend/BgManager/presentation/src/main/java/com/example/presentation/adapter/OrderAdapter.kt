package com.example.presentation.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.example.presentation.databinding.ItemlayoutOrderBinding
import com.example.presentation.ui.OrderUiState

//class OrderAdapter(private val itemList: List<OrderUiState>) : RecyclerView.Adapter<OrderAdapter.OrderViewHolder>() {
class OrderAdapter : ListAdapter<OrderUiState, OrderAdapter.OrderViewHolder>(OrderDiffCallback()) {

    class OrderViewHolder(private val binding: ItemlayoutOrderBinding) : RecyclerView.ViewHolder(binding.root) {

        fun bind(item: OrderUiState) {
            binding.orderItem = item
            binding.executePendingBindings()
            println("어댑터 뷰홀더 들어옴 ${binding.orderItem}")
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): OrderViewHolder {
        val inflater = LayoutInflater.from(parent.context)
        val binding = ItemlayoutOrderBinding.inflate(inflater, parent, false)
        return OrderViewHolder(binding)
    }

    override fun onBindViewHolder(holder: OrderViewHolder, position: Int) {
//        val item = itemList[position]
        val item = getItem(position)
        println("바인드 뷰홀더 들어옴 $item")
        holder.bind(item)
    }

//    override fun getItemCount(): Int {
//        return itemList.size
//    }

    private class OrderDiffCallback : DiffUtil.ItemCallback<OrderUiState>() {
        override fun areItemsTheSame(oldItem: OrderUiState, newItem: OrderUiState): Boolean {
            return oldItem.orderId == newItem.orderId
        }

        override fun areContentsTheSame(oldItem: OrderUiState, newItem: OrderUiState): Boolean {
            return oldItem == newItem
        }
    }

}