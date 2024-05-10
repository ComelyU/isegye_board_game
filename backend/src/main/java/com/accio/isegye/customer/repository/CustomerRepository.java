package com.accio.isegye.customer.repository;

import com.accio.isegye.customer.entity.Customer;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

@Repository
public interface CustomerRepository extends JpaRepository<Customer, Integer> {

    @Query("SELECT SUM(d.totalPrice)\n"
        + " FROM Customer c\n"
        + " LEFT JOIN OrderMenu m ON c.id = m.customer.id\n"
        + " LEFT JOIN OrderMenuDetail d ON m.id = d.orderMenu.id\n"
        + " WHERE c.id = ?1")
    Integer getMenuFeeByCustomerId(int customerId);

    @Query("select c "
        + "from OrderGame og "
        + "left join Customer c on og.customer.id = c.id "
        + "where og.id = ?1")
    Customer findCustomerByOrderGameId(long orderGameId);
}
