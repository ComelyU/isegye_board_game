package com.example.presentation.adapter

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.example.presentation.databinding.ItemlayoutOrderBinding
import com.example.presentation.ui.OrderUiState

class OrderAdapter(private val orderOnClickListener: OrderOnClickListener)
    : ListAdapter<OrderUiState, OrderAdapter.OrderViewHolder>(OrderDiffCallback()) {

    class OrderViewHolder(private val binding: ItemlayoutOrderBinding, private val clickListener: OrderOnClickListener
    ) : RecyclerView.ViewHolder(binding.root) {

        fun bind(item: OrderUiState) {
            binding.orderItem = item
            binding.cartButton.setOnClickListener {
                clickListener.onOrderClicked(item.orderId, item.roomNumber)
            }
            binding.executePendingBindings()
//            println("어댑터 뷰홀더 들어옴 ${binding.orderItem}")

            val adapter = OrderDetailAdapter(item.orderDetail)
            binding.orderDetailsRV.adapter = adapter
        }
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): OrderViewHolder {
        val inflater = LayoutInflater.from(parent.context)
        val binding = ItemlayoutOrderBinding.inflate(inflater, parent, false)
        return OrderViewHolder(binding, orderOnClickListener)
    }

    override fun onBindViewHolder(holder: OrderViewHolder, position: Int) {
//        val item = itemList[position]
        val item = getItem(position)
//        println("바인드 뷰홀더 들어옴 $item")
        holder.bind(item)
    }

    private class OrderDiffCallback : DiffUtil.ItemCallback<OrderUiState>() {
        override fun areItemsTheSame(oldItem: OrderUiState, newItem: OrderUiState): Boolean {
            return oldItem.orderId == newItem.orderId
        }

        override fun areContentsTheSame(oldItem: OrderUiState, newItem: OrderUiState): Boolean {
            return oldItem == newItem
        }
    }

    interface OrderOnClickListener {
        fun onOrderClicked(orderId: Int, roomNumber: Int)
    }
}